function val_sim_input(input)

    N = input.robot.n_joints;

    assert(length(input.robot.joint_types) == N, 'joint_types size mismatch');
    assert(length(input.robot.a) == N, 'a size mismatch');
    assert(length(input.robot.alpha) == N, 'alpha size mismatch');
    assert(length(input.robot.d) == N, 'd size mismatch');
    assert(length(input.robot.theta) == N, 'theta size mismatch');

    assert(length(input.robot.masses) == N, 'masses size mismatch');
    assert(size(input.robot.COM,1) == N && size(input.robot.COM,2) == 3, 'COM must be Nx3');
    assert(size(input.robot.inertia,1) == N && size(input.robot.inertia,2) == 3, 'inertia must be Nx3');

    assert(length(input.sim.q0) == N, 'q0 mismatch');
    assert(length(input.sim.qd0) == N, 'qd0 mismatch');

    assert(length(input.sim.external_tau) == N, 'external_tau mismatch');

    if strcmp(input.control.mode, 'Compliance')
        assert(length(input.control.kp) == N, 'kp mismatch');
        assert(length(input.control.kd) == N, 'kd mismatch');
    end

    if strcmp(input.control.mode, 'Impedance')
        assert(any(length(input.control.Md) == [1 3 6 N]), 'Md mismatch');
        assert(any(length(input.control.Bd) == [1 3 6 N]), 'Bd mismatch');
        assert(any(length(input.control.Kd) == [1 3 6 N]), 'Kd mismatch');
    end

end
