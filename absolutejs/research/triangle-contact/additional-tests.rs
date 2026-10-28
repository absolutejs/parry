
#[test]
fn frames_survive_old_identity_cache_and_internal_edge_constraints() {
    use parry3d::shape::HeightFieldFlags;
    for state in ["before", "after"] {
        for flipped in [false, true] {
            for internal_edges in [false, true] {
                let (mut h, c, p, prediction, id) = fixture(state);
                if internal_edges { h.set_flags(HeightFieldFlags::FIX_INTERNAL_EDGES); }
                let mut manifolds: Vec<ContactManifold<(), u32>> = Vec::new();
                let mut workspace: Option<ContactManifoldsWorkspace> = None;
                for iteration in 0..4 {
                    if flipped { DefaultQueryDispatcher.contact_manifolds(&p.inverse(), &c, &h, prediction, &mut manifolds, &mut workspace).unwrap(); }
                    else { DefaultQueryDispatcher.contact_manifolds(&p, &h, &c, prediction, &mut manifolds, &mut workspace).unwrap(); }
                    let m = manifolds.iter().find(|m| if flipped {m.subshape2 == id} else {m.subshape1 == id}).unwrap();
                    let frame = if flipped {m.subshape_pos2()} else {m.subshape_pos1()}.expect("explicit triangle frame");
                    assert_eq!(frame.translation, h.triangle_at_id(id).unwrap().a);
                    let limit = if state == "before" {0.001} else {0.02};
                    for contact in &m.points {
                        assert!(contact.dist >= -limit, "state={state} flipped={flipped} edges={internal_edges} iteration={iteration}: {}",contact.dist);
                        let a = m.subshape_pos1().map(|s| s.transform_point(contact.local_p1)).unwrap_or(contact.local_p1);
                        let b = m.subshape_pos2().map(|s| s.transform_point(contact.local_p2)).unwrap_or(contact.local_p2);
                        let relative = if flipped {p.inverse()} else {p};
                        let normal = m.subshape_pos1().map(|s| s.rotation * m.local_n1).unwrap_or(m.local_n1);
                        let reconstructed = (relative.transform_point(b) - a).dot(normal);
                        assert!((reconstructed-contact.dist).abs() < 0.001, "world/collider frame mismatch");
                    }
                    // Simulate a restored cache in the previous identity coordinate frame.
                    // Neither feature IDs nor per-contact payloads are changed by this conversion.
                    for m in &mut manifolds {
                        let frame = if flipped {m.subshape_pos2()} else {m.subshape_pos1()}.copied().unwrap();
                        for contact in &mut m.points {
                            if flipped {contact.local_p2 = frame.transform_point(contact.local_p2);}
                            else {contact.local_p1 = frame.transform_point(contact.local_p1);}
                            contact.data = 42;
                        }
                        if flipped {m.set_subshape_pos2(None);} else {m.set_subshape_pos1(None);}
                    }
                    let bytes = bincode::serialize(&(manifolds,workspace)).unwrap();
                    (manifolds,workspace)=bincode::deserialize(&bytes).unwrap();
                    assert_eq!(bytes,bincode::serialize(&(&manifolds,&workspace)).unwrap());
                }
            }
        }
    }
}
