function [xout, yout] = get_position_tri(x1, y1, r1, x2, y2, r2, x3, y3, r3)

    % get each duo of intersection position (could be NaN in no intersection --> test that)
    [xout12, yout12] = circcirc(x1, y1, r1, x2, y2, r2);
    [xout13, yout13] = circcirc(x1, y1, r1, x3, y3, r3);
    [xout23, yout23] = circcirc(x3, y3, r3, x2, y2, r2);

    % test if we have three intersection
    if isnan(xout12(1)) || isnan(xout13(1)) || isnan(xout23(1)) || isnan(xout12(2)) || isnan(xout13(2)) || isnan(xout23(2))
        xout = NaN;
        yout = NaN;
        return;
    end

    % we don't, compute sum of distances between points
    sum_vect = zeros(8, 1);
    for i=1: 8
        if sum(i == [1, 2, 3, 4])   % test if we use first or second intersection for 12
            ind1 = 1;
        else 
            ind1 = 2;
        end
        if sum(i == [1, 2, 5, 6])   % test if we use first or second intersection for 13
            ind2 = 1;
        else 
            ind2 = 2;
        end
        if sum(i == [2, 4, 6, 8])   % test if we use first or second intersection for 23
            ind3 = 2;
        else 
            ind3 = 1;
        end

        % get sum of three distances between points
        sum_vect(i) = norm([xout12(ind1) - xout13(ind2), yout12(ind1) - yout13(ind2)]) + ...
                    norm([xout12(ind1) - xout23(ind3), yout12(ind1) - yout23(ind3)]) + ...
                    norm([xout23(ind3) - xout13(ind2), yout23(ind3) - yout13(ind2)]);
    end

    % find minimum and get corresponding indexes
    [~, index] = min(sum_vect);
    switch index
        case 1
            ind1 = 1; ind2 = 1; ind3 = 1;
        case 2
            ind1 = 1; ind2 = 1; ind3 = 2;
        case 3
            ind1 = 1; ind2 = 2; ind3 = 1;
        case 4
            ind1 = 1; ind2 = 2; ind3 = 2;
        case 5
            ind1 = 2; ind2 = 1; ind3 = 1;
        case 6
            ind1 = 2; ind2 = 1; ind3 = 2;
        case 7
            ind1 = 2; ind2 = 2; ind3 = 1;
        case 8
            ind1 = 2; ind2 = 2; ind3 = 2;
    end

    % return mean with computed indexes
    xout = mean([xout12(ind1), xout13(ind2), xout23(ind3)]);
    yout = mean([yout12(ind1), yout13(ind2), yout23(ind3)]);

end