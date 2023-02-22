function accessibilityMap = getAccessibilityMap(alpha,lightStrength,lightPos,obstacle,accessibilityMap,fac)
    % alpha is the decay factor, can kill light exponentially if < 1
    % lightStrength = initial starting light value, set to 1, but can be
    % changed
    % obstacle is actually the complement of occupancy grid
    % accessibility map = empty place holder that is filled and returned
    % fac = factor governing curves as seen in a_quiver_plots.m
    nx = size(accessibilityMap,1);
    ny = size(accessibilityMap,2);
    %% 1
    max_col = nx-lightPos(1);
    max_row = ny-lightPos(2);
    for i = 0:max_col
        currentX = lightPos(1) + i;
        for j = 0:max_row
            currentY = lightPos(2) + j;
            if i == 0 && j == 0
                accessibilityMap(currentX,currentY) = lightStrength;
            elseif i == 0
                accessibilityMap(currentX,currentY) = alpha * accessibilityMap(currentX,currentY-1);
            elseif j == 0
                accessibilityMap(currentX,currentY) = alpha * accessibilityMap(currentX-1,currentY);
            elseif i == j*fac
                accessibilityMap(currentX,currentY) = alpha * accessibilityMap(currentX-1,currentY-1);
            elseif i > j*fac
                c = ((currentY-lightPos(2))*fac)/(currentX-lightPos(1));
                f = accessibilityMap(currentX-1,currentY) - c * (accessibilityMap(currentX-1,currentY)-accessibilityMap(currentX-1,currentY-1));
                accessibilityMap(currentX,currentY) = alpha * f;
            elseif j*fac > i
                c = (currentX-lightPos(1))/((currentY-lightPos(2))*fac);
                f = accessibilityMap(currentX,currentY-1) - c * (accessibilityMap(currentX,currentY-1) - accessibilityMap(currentX-1,currentY-1));
                accessibilityMap(currentX,currentY) = alpha * f;
            end
            accessibilityMap(currentX,currentY) = accessibilityMap(currentX,currentY) * obstacle(currentX,currentY);
        end
    end
    %% 2
    max_col = lightPos(1);
    max_row = ny-lightPos(2);
    for i = 0:max_col - 1
        currentX = lightPos(1) - i;
        for j = 0:max_row
            currentY = lightPos(2) + j;
            if i == 0 && j == 0
                accessibilityMap(currentX,currentY) = lightStrength;
            elseif i == 0
                accessibilityMap(currentX,currentY) = alpha * accessibilityMap(currentX,currentY-1);
            elseif j == 0
                accessibilityMap(currentX,currentY) = alpha * accessibilityMap(currentX+1,currentY);
            elseif i == j*fac
                accessibilityMap(currentX,currentY) = alpha * accessibilityMap(currentX+1,currentY-1);
            elseif i > j*fac
                c = ((currentY-lightPos(2))*fac)/(currentX-lightPos(1));
                f = accessibilityMap(currentX+1,currentY) + c * (accessibilityMap(currentX+1,currentY)-accessibilityMap(currentX+1,currentY-1));
                accessibilityMap(currentX,currentY) = alpha * f;
            elseif j*fac > i
                c = (currentX-lightPos(1))/((currentY-lightPos(2))*fac);
                f = accessibilityMap(currentX,currentY-1) + c * (accessibilityMap(currentX,currentY-1) - accessibilityMap(currentX+1,currentY-1));
                accessibilityMap(currentX,currentY) = alpha * f;
            end
            accessibilityMap(currentX,currentY) = accessibilityMap(currentX,currentY) * obstacle(currentX,currentY);
        end
    end
    %% 3
    max_col = lightPos(1);
    max_row = lightPos(2);
    for i = 0:max_col - 1
        currentX = lightPos(1) - i;
        for j = 0:max_row - 1
            currentY = lightPos(2) - j;
            if i == 0 && j == 0
                accessibilityMap(currentX,currentY) = lightStrength;
            elseif i == 0
                accessibilityMap(currentX,currentY) = alpha * accessibilityMap(currentX,currentY+1);
            elseif j == 0
                accessibilityMap(currentX,currentY) = alpha * accessibilityMap(currentX+1,currentY);
            elseif i == j*fac
                accessibilityMap(currentX,currentY) = alpha * accessibilityMap(currentX+1,currentY+1);
            elseif i > j*fac
                c = ((currentY-lightPos(2))*fac)/(currentX-lightPos(1));
                f = accessibilityMap(currentX+1,currentY) - c * (accessibilityMap(currentX+1,currentY)-accessibilityMap(currentX+1,currentY+1));
                accessibilityMap(currentX,currentY) = alpha * f;
            elseif j*fac > i
                c = (currentX-lightPos(1))/((currentY-lightPos(2))*fac);
                f = accessibilityMap(currentX,currentY+1) - c * (accessibilityMap(currentX,currentY+1) - accessibilityMap(currentX+1,currentY+1));
                accessibilityMap(currentX,currentY) = alpha * f;
            end
            accessibilityMap(currentX,currentY) = accessibilityMap(currentX,currentY) * obstacle(currentX,currentY);
        end
    end

    %% 4
    max_col = nx-lightPos(1);
    max_row = lightPos(2);
    for i = 0:max_col
        currentX = lightPos(1) + i;
        for j = 0:max_row - 1
            currentY = lightPos(2) - j;
            if i == 0 && j == 0
                accessibilityMap(currentX,currentY) = lightStrength;
            elseif i == 0
                accessibilityMap(currentX,currentY) = alpha * accessibilityMap(currentX,currentY+1);
            elseif j == 0
                accessibilityMap(currentX,currentY) = alpha * accessibilityMap(currentX-1,currentY);
            elseif i == j*fac
                accessibilityMap(currentX,currentY) = alpha * accessibilityMap(currentX-1,currentY+1);
            elseif i > j*fac
                c = ((currentY-lightPos(2))*fac)/(currentX-lightPos(1));
                f = accessibilityMap(currentX-1,currentY) + c * (accessibilityMap(currentX-1,currentY)-accessibilityMap(currentX-1,currentY+1));
                accessibilityMap(currentX,currentY) = alpha * f;
            elseif j*fac > i
                c = (currentX-lightPos(1))/((currentY-lightPos(2))*fac);
                f = accessibilityMap(currentX,currentY+1) + c * (accessibilityMap(currentX,currentY+1) - accessibilityMap(currentX-1,currentY+1));
                accessibilityMap(currentX,currentY) = alpha * f;
            end
            accessibilityMap(currentX,currentY) = accessibilityMap(currentX,currentY) * obstacle(currentX,currentY);
        end
    end
end