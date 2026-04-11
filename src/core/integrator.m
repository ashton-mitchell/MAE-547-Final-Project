function [q_next, qd_next] = integrator(q, qd, qdd, dt)
    q_next = q + qd * dt;
    qd_next = qd + qdd * dt;
end