% Place the csv files in ./data directory.

function ICRAsim
    % Variables
    % thetahat -> estimated ARX model
    % phi -> new and past y and u signals stacked
    % ehat -> Error term
    % L -> Gain matrix
    % C -> Covariance

    filelist = dir('data/*.csv'); % Data location
    stable_limit = 10; % Number of steps that the model error should be less than threshold to consider stability
    epsilon = 0.002; % Threshold for detection if the model is stable for fault detection
    epsilon_2 = 0.001; % Threshold for detection if the variance is stable for z-score calculation
    na = 5; % Number of past output samples
    nb = 5; % Number of past input samples

    % Counter for various metrics
    fp = 0;
    tp = 0;
    fn = 0;
    tn = 0;
    detection_times = [];
    to_print = 1; % Set 0 if plots are not required

    for filenum = 1:size(filelist, 1)
        filename = strcat('data/', filelist(filenum).name);
        data = csvread(filename);

        zlist = zeros(size(data, 1), 6);

        for j = 1:6
            stable_for_detection = 0;
            stable_counter = 0;
            stable_counter_var = 0;
            stable_variance = 0;
            stable_anomaly = 0;
            ehatmean = 0;
            after_stable_counter = 1;

            thetahat = zeros(na + nb, 1);
            C = eye(na + nb);
            ylist = zeros(na, 1);
            ulist = zeros(nb, 1);

            for i = 1:size(data, 1)
                y = data(i, j+3);
                ylist = [y; ylist(1:end - 1)];
                ulist = [data(i, j+12); ulist(1:end - 1)];
                phi = [ylist; ulist];

                ehat = y - phi'*thetahat;
                L = C*phi/(1 + phi'*C*phi);
                thetahat = thetahat + L*ehat;
                C = C - L*phi'*C;

                if (norm(L*ehat, Inf) < epsilon) && (stable_for_detection == 0)
                    if stable_counter < stable_limit
                        stable_counter = stable_counter + 1;
                    else
                        stable_for_detection = 1;
                    end
                end

                if stable_for_detection
                    ehatmeanprev = ehatmean;
                    after_stable_counter = after_stable_counter + 1;
                    ehatmean = ehatmean*(after_stable_counter - 1)/after_stable_counter + ehat/after_stable_counter;
                    stable_variance = stable_variance + (ehat - ehatmeanprev)'*(ehat - ehatmean);

                    sn = sqrt(stable_variance/(after_stable_counter - 1));

                    if ((ehat - ehatmeanprev)'*(ehat - ehatmean) < epsilon_2) && (stable_anomaly == 0)
                        if stable_counter_var < stable_limit
                            stable_counter_var = stable_counter_var + 1;
                        else
                            stable_anomaly = 1;
                        end
                    end

                    if stable_anomaly
                        z = abs((ehat - ehatmean)/sn);
                        zlist(i, j) = z;
                    end
                end

            end
        end

        fault_time = 4.5*(data(:, 20) | data(:, 21) | data(:, 22) | data(:, 23) | data(:, 24));
        threshold = 4.5*(ones(size(data, 1), 1));
        fault_inst = 0.25*find(fault_time, 1);
        if isempty(fault_inst)
            fault_inst = Inf;
        end
        [detect_inst, detect_list] = find(zlist > 4.5);
        detect_inst = detect_inst*0.25;

        fp = fp + any(detect_inst < fault_inst);
        tn = tn + 1 - any(detect_inst < fault_inst);

        if isfinite(fault_inst)
            tp = tp + any(detect_inst >= fault_inst);
            fn = fn + 1 - any(detect_inst >= fault_inst);
            detection_time = (detect_inst - fault_inst);
            detection_time = detection_time(find(detection_time > 0, 1));
            detection_times = [detection_time; detection_times];
        end

        if to_print
            csvwrite(strcat(filename(6:end-4), '_output.csv'), [(0:0.25:0.25*(size(data, 1) - 1))', zlist, fault_time])

            h = figure();

            subplot(3, 2, 1)
            hold on
            box on
            plot(0:0.25:0.25*(size(data, 1) - 1), zlist(:, 1))
            plot(0:0.25:0.25*(size(data, 1) - 1), threshold, 'g')
            plot(0:0.25:0.25*(size(data, 1) - 1), fault_time)
            title("v_x")
            xlabel("Time (s)")
            ylabel("Z-score")
            axis([0 0.25*(size(data, 1) - 1) 0 6])

            subplot(3, 2, 2)
            hold on
            box on
            plot(0:0.25:0.25*(size(data, 1) - 1), zlist(:, 2))
            plot(0:0.25:0.25*(size(data, 1) - 1), threshold, 'g')
            plot(0:0.25:0.25*(size(data, 1) - 1), fault_time)
            title("v_y")
            xlabel("Time (s)")
            ylabel("Z-score")
            axis([0 0.25*(size(data, 1) - 1) 0 6])

            subplot(3, 2, 3)
            hold on
            box on
            plot(0:0.25:0.25*(size(data, 1) - 1), zlist(:, 3))
            plot(0:0.25:0.25*(size(data, 1) - 1), threshold, 'g')
            plot(0:0.25:0.25*(size(data, 1) - 1), fault_time)
            title("v_z")
            xlabel("Time (s)")
            ylabel("Z-score")
            axis([0 0.25*(size(data, 1) - 1) 0 6])

            subplot(3, 2, 4)
            hold on
            box on
            plot(0:0.25:0.25*(size(data, 1) - 1), zlist(:, 4))
            plot(0:0.25:0.25*(size(data, 1) - 1), threshold, 'g')
            plot(0:0.25:0.25*(size(data, 1) - 1), fault_time)
            title("roll")
            xlabel("Time (s)")
            ylabel("Z-score")
            axis([0 0.25*(size(data, 1) - 1) 0 6])

            subplot(3, 2, 5)
            hold on
            box on
            plot(0:0.25:0.25*(size(data, 1) - 1), zlist(:, 5))
            plot(0:0.25:0.25*(size(data, 1) - 1), threshold, 'g')
            plot(0:0.25:0.25*(size(data, 1) - 1), fault_time)
            title("pitch")
            xlabel("Time (s)")
            ylabel("Z-score")
            axis([0 0.25*(size(data, 1) - 1) 0 6])

            subplot(3, 2, 6)
            hold on
            box on
            plot(0:0.25:0.25*(size(data, 1) - 1), zlist(:, 6))
            plot(0:0.25:0.25*(size(data, 1) - 1), threshold, 'g')
            plot(0:0.25:0.25*(size(data, 1) - 1), fault_time)
            title("yaw")
            xlabel("Time (s)")
            ylabel("Z-score")
            axis([0 0.25*(size(data, 1) - 1) 0 6])

            savename = strcat(filename(6:end-4), '.png');
            print(gcf, '-dpng', savename);
        end
    end

    fp
    tp
    fn
    tn
    mean_detection_time = mean(detection_times)
    % waitfor(h)

end
