function [accessibilityMap,lightSource_enum] = getAccessibilityMapPlanner(alpha,lightStrength,lightPos,obstacle,accessibilityMap,lightSource_enum,iter,threshold)
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
            elseif i == j
                accessibilityMap(currentX,currentY) = alpha * accessibilityMap(currentX-1,currentY-1);
            elseif i > j
                c = (currentY-lightPos(2))/(currentX-lightPos(1));
                f = accessibilityMap(currentX-1,currentY) - c * (accessibilityMap(currentX-1,currentY)-accessibilityMap(currentX-1,currentY-1));
                accessibilityMap(currentX,currentY) = alpha * f;
            elseif j > i
                c = (currentX-lightPos(1))/(currentY-lightPos(2));
                f = accessibilityMap(currentX,currentY-1) - c * (accessibilityMap(currentX,currentY-1) - accessibilityMap(currentX-1,currentY-1));
                accessibilityMap(currentX,currentY) = alpha * f;
            end
            v = accessibilityMap(currentX,currentY) * obstacle(currentX,currentY);
            accessibilityMap(currentX,currentY) = v;
            if v >= threshold
                if isnan(lightSource_enum(currentX, currentY))
                    lightSource_enum(currentX, currentY) = iter;
                end
            end
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
            elseif i == j
                accessibilityMap(currentX,currentY) = alpha * accessibilityMap(currentX+1,currentY-1);
            elseif i > j
                c = (currentY-lightPos(2))/(currentX-lightPos(1));
                f = accessibilityMap(currentX+1,currentY) + c * (accessibilityMap(currentX+1,currentY)-accessibilityMap(currentX+1,currentY-1));
                accessibilityMap(currentX,currentY) = alpha * f;
            elseif j > i
                c = (currentX-lightPos(1))/(currentY-lightPos(2));
                f = accessibilityMap(currentX,currentY-1) + c * (accessibilityMap(currentX,currentY-1) - accessibilityMap(currentX+1,currentY-1));
                accessibilityMap(currentX,currentY) = alpha * f;
            end
            v = accessibilityMap(currentX,currentY) * obstacle(currentX,currentY);
            accessibilityMap(currentX,currentY) = v;
            if v >= threshold
                if isnan(lightSource_enum(currentX, currentY))
                    lightSource_enum(currentX, currentY) = iter;
                end
            end
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
            elseif i == j
                accessibilityMap(currentX,currentY) = alpha * accessibilityMap(currentX+1,currentY+1);
            elseif i > j
                c = (currentY-lightPos(2))/(currentX-lightPos(1));
                f = accessibilityMap(currentX+1,currentY) - c * (accessibilityMap(currentX+1,currentY)-accessibilityMap(currentX+1,currentY+1));
                accessibilityMap(currentX,currentY) = alpha * f;
            elseif j > i
                c = (currentX-lightPos(1))/(currentY-lightPos(2));
                f = accessibilityMap(currentX,currentY+1) - c * (accessibilityMap(currentX,currentY+1) - accessibilityMap(currentX+1,currentY+1));
                accessibilityMap(currentX,currentY) = alpha * f;
            end
            v = accessibilityMap(currentX,currentY) * obstacle(currentX,currentY);
            accessibilityMap(currentX,currentY) = v;
            if v >= threshold
                if isnan(lightSource_enum(currentX, currentY))
                    lightSource_enum(currentX, currentY) = iter;
                end
            end
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
            elseif i == j
                accessibilityMap(currentX,currentY) = alpha * accessibilityMap(currentX-1,currentY+1);
            elseif i > j
                c = (currentY-lightPos(2))/(currentX-lightPos(1));
                f = accessibilityMap(currentX-1,currentY) + c * (accessibilityMap(currentX-1,currentY)-accessibilityMap(currentX-1,currentY+1));
                accessibilityMap(currentX,currentY) = alpha * f;
            elseif j > i
                c = (currentX-lightPos(1))/(currentY-lightPos(2));
                f = accessibilityMap(currentX,currentY+1) + c * (accessibilityMap(currentX,currentY+1) - accessibilityMap(currentX-1,currentY+1));
                accessibilityMap(currentX,currentY) = alpha * f;
            end
            v = accessibilityMap(currentX,currentY) * obstacle(currentX,currentY);
            accessibilityMap(currentX,currentY) = v;
            if v >= threshold
                if isnan(lightSource_enum(currentX, currentY))
                    lightSource_enum(currentX, currentY) = iter;
                end
            end
        end
    end
end