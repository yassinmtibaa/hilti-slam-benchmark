function tform = pcregisterloam(moving, fixed, varargin)
    % Parse input parameters
    parser = inputParser;
    parser.addParameter('InitialTransform', rigidtform3d);
    parser.addParameter('GridStep', 0.4);
    parser.parse(varargin{:});
    params = parser.Results;
    
    % Downsample point clouds
    fixedDown = pcdownsample(fixed, 'gridAverage', params.GridStep);
    movingDown = pcdownsample(moving, 'gridAverage', params.GridStep);
    
    % Get calibration transform (LiDAR to camera)
    T_t = getVeloToCam();
    
    % Transform to camera coordinate frame
    fixedCam = transformPC(fixedDown.Location', T_t)';
    movingCam = transformPC(movingDown.Location', T_t)';
    
    % Apply initial transform if provided
    if ~isempty(params.InitialTransform)
        initMatrix = [params.InitialTransform.R, params.InitialTransform.Translation'; 0 0 0 1];
        movingCam = transformPC(movingCam', initMatrix)';
    end
    
    % LOAM optimization parameters
    theta = zeros(6, 1);  % [tx, ty, tz, rx, ry, rz]
    lambda = 0.001;
    maxIter = 10;
    n_max_features = 100;
    
    % Optimization loop
    for iter = 1:maxIter
        % Apply current transformation
        R = getR(theta(4), theta(5), theta(6));
        t = theta(1:3);
        movingTransformed = (R * movingCam' + t)';
        
        % Find planar features
        [normals, points_fixed, points_moving] = ...
            findPlanarFeatures(fixedCam, movingTransformed, movingCam, n_max_features);
        
        % Break if not enough features
        if numel(normals) < 5
            warning('Insufficient features for registration');
            break;
        end
        
        % Compute residuals and Jacobian
        [J, residuals] = computePlaneResiduals(normals, points_fixed, points_moving, theta);
        
        % Levenberg-Marquardt update
        H = J' * J;
        H_lm = H + lambda * diag(diag(H));
        delta_theta = -H_lm \ (J' * residuals);
        
        % Update parameters
        theta = theta + delta_theta;
    end
    
    % Convert to rigidtform3d
    R_final = getR(theta(4), theta(5), theta(6));
    t_final = theta(1:3)';
    tform = rigidtform3d(R_final, t_final);
end
