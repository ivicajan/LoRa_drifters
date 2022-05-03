clc; clear all; close all;
%T = importdata('sample2.csv');
samples = readtable('svtD01_IMU.csv');
date = samples.Date;
time_tot = milliseconds(samples.Time); %millis
%Start-End test time
start_hr = 2; 
start_min = 10; %2:10
end_hr = 3;
end_min = 50; %3:50
time_range=[];
[~,time_range(1)]=min(abs(time_tot(:,1)-(start_hr*60+start_min)*60*1000)); %get valid data time range
[~,time_range(2)]=min(abs(time_tot(:,1)-(end_hr*60+end_min)*60*1000));

%get valid data
input(:,1)=milliseconds(samples.Time([time_range(1):time_range(2)])); %millis
input(:,[2:7])=table2array(samples([time_range(1):time_range(2)],[11:16])); %acc and rot matrix
nanIDX = find(isnan(input));
while(~isempty(nanIDX)) %replace nan with neast value
  input(nanIDX) = input(nanIDX-1);
  nanIDX      = find(isnan(input));
end 
x_ins_esp=table2array(samples([time_range(1):time_range(2)],[3:7])); %X_INS from esp
y_ins_esp=table2array(samples([time_range(1):time_range(2)],[9:10])); %Y_GPS from esp
u_gps_esp=table2array(samples([time_range(1):time_range(2)],[17:18])); %updated GPS from esp

%for gps raw
gps_samples = readtable('svtD01.csv');
t_gps_raw = milliseconds(table2array(gps_samples(:,2)));
[~,time_range_2(1)]=min(abs(t_gps_raw(:,1)-(start_hr*60+start_min)*60*1000));
[~,time_range_2(2)]=min(abs(t_gps_raw(:,1)-(end_hr*60+end_min)*60*1000));
raw_gps_time = milliseconds(table2array(gps_samples([time_range_2(1):time_range_2(2)],2)));
raw_gps = table2array(gps_samples([time_range_2(1):time_range_2(2)],[3:4]));
%%Raw gps lat lng to meters
raw_lat = -(raw_gps(:,2)-raw_gps(1,2)) .* (6372795* pi) ./180;
raw_lng = (raw_gps(:,1)-raw_gps(1,1)) .* (6372795* pi) ./180;

T = 1; %usually 0.15s
beta = 0.005;
fix_ = 0;
G = 1;  %Gravity, if involved already, set as 1; otherwize set as 9.81
X_INS=zeros(5,1);
Y_GPS = [raw_gps(1,2); raw_gps(11); input(1,7)]
Bias_pre = zeros(3,1);
H = [zeros(3,2) eye(3) zeros(3)];
e_r = 1.1; %error of GPS, larger means less depends 
e_q = 0.1; %error of imu, larger means less depends 
i_p = 10; %indext of P0
P_0 =diag([i_p,i_p,i_p,i_p,0.2/pi,i_p/2,i_p/2,25*3.14/180]).^2;
R=diag([e_r,e_r,e_r*3.14/180]).^2;
Q=diag([e_q,e_q,e_q*3.14/180]).^2;
P = P_0;

X_cal(1,:) = [0; 0; 0; 0];
output_nok= zeros(1,5);

%Loop: 
[n1, n2] = size(input);

for i = 2:1:n1 %size write by hand
    %get data
    U = [input(i,2)*G;input(i,3)*G;(input(i,7)-input(i-1,7))/T];
    Y_GPS = [raw_lat(i); raw_lng(i); input(i,7)];
    
    %check if stable
    error = 0.01;
    if abs(sqrt(input(i,2)^2+input(i,3)^2+input(i,4)^2)-1) < error
        fix_ = fix_ + 1;
    else 
        fix_ = 0;
    end 
    
    C = [cos(Y_GPS(3)), sin(Y_GPS(3)); -sin(Y_GPS(3)), cos(Y_GPS(3))];
    A_E=[ [zeros(2);eye(2);zeros(4,2)] zeros(8,3) [C; zeros(6,2)] [zeros(4,1);1;zeros(3,1)] ];
    B_E = [[C;zeros(6,2)] [zeros(4,1);1;zeros(3,1)] ];
    
    %Ouput Kalman
    U_k = U - Bias_pre;
    X_INS_ = X_INS;
    X_INS = [X_INS_(1)+ T*U_k(1); X_INS_(2)+ T*U_k(2); 0.5*T*X_INS_(1)+X_INS_(3); 0.5*T*X_INS_(2)+X_INS_(4); X_INS_(5)+T*U_k(3) ];
    P = A_E*P*transpose(A_E) + B_E*Q*transpose(B_E) + P_0*beta;
    
    %Update Kalman
    Y_E = X_INS([3:5])-Y_GPS;
    G_k = P*transpose(H) * inv(H*P*transpose(H)+R);
    P=(eye(8)-G_k*H)*P;
    X_E_Pre=[zeros(5,1);Bias_pre] + G_k*Y_E;
    IMU_SPEED_RATIO = 0.8;
    X_E_Pre([1:2]) = [X_INS(1);X_INS(2)]*(1-IMU_SPEED_RATIO);
    X_INS = X_INS - X_E_Pre([1:5]);
    Bias_pre = X_E_Pre([6:8]);
    
    
    x_rot = C*U([1,2]);
    output(i,:) = [X_INS(1),X_INS(2),X_INS(3),X_INS(4),X_INS(5)];
    
    output_nok(i,:) = [output_nok(1)+ T*x_rot(1); output_nok(2)+ T*x_rot(2); 0.5*T*T*x_rot(1)+output_nok(3); 0.5*T*T*x_rot(2)+output_nok(4); output_nok(5)+T*U_k(3) ];
end 
output(1,:)=output(2,:); %eliminate fisrts

figure, 
subplot(3,2,1); plot(output(:,3), output(:,4)); title('xy distance kalman'); xlabel('x (m)'); ylabel('y (m)');
subplot(3,2,2); plot(output(:,1), output(:,2)); title('xy speed kalman'); xlabel('x (m/s)'); ylabel('y (m/s)');
subplot(3,2,3); plot(input(:,1), (output(:,5))-input(:,7)); title('error of Yaw');
subplot(3,2,5); plot(output_nok(:,3), output_nok(:,4)); title('xy distance simple intigration'); xlabel('x (m)'); ylabel('y (m)');
subplot(3,2,6); plot(output_nok(:,1), output_nok(:,2)); title('xy speed simple intigration'); xlabel('x (m/s)'); ylabel('y (m/s)');
figure, plot3(input(:,1), output(:,3), output(:,4)); xlabel('time (sec)'); ylabel('x (m)'); zlabel('y (m)');
% plot3(input(:,1), x_ins_esp(:,3), x_ins_esp(:,4));title('xy distance by ESP32'); xlabel('time (sec)'); ylabel('x (m)'); zlabel('y (m)');
hold on;
plot3(raw_gps_time(:,1), raw_lat, raw_lng);title('xy distance by raw gps'); xlabel('time (sec)'); ylabel('x (m)'); zlabel('y (m)');
figure, 
plot(output(:,3), output(:,4)); 
hold on
plot(raw_lat, raw_lng); title('xy distance, R=1 Q=0.1, decrease speed to 1%'); xlabel('x (m)'); ylabel('y (m)');legend('Kalman', 'GPS');
figure, plot(x_ins_esp(:,3), x_ins_esp(:,2)); title('xy distance, R=1.1 Q=0.1, esp'); xlabel('x (m)');ylabel('y (m)');