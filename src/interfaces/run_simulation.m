function output = run_simulation(input)

    switch input.control.mode

        case 'Dynamics'
            output = run_dynamics(input);

        case 'Compliance'
            output = run_compliance(input);

        case 'Impedance'
            output = run_impedance(input);

        otherwise
            error('Unknown control mode');
    end

end