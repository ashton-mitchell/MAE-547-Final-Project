function col = safe_copy(data, colIdx, N)

    col = zeros(N,1);

    for i = 1:N
        val = data{i, colIdx};

        if isempty(val)
            col(i) = 0;
        elseif isnumeric(val)
            col(i) = val;
        elseif ischar(val) || isstring(val)
            num = str2double(val);
            if isnan(num)
                col(i) = 0;
            else
                col(i) = num;
            end
        else
            col(i) = 0;
        end
    end

end