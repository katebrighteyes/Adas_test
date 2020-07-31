clear;

Ts = 1;  
N_samples = 100;
Rm_v = 6;
Rm_r = 5;

Xp1 = zeros(N_samples,1); Yp1 = zeros(N_samples,1);   
Xp2 = zeros(N_samples,1); Yp2 = zeros(N_samples,1);
Xp3 = zeros(N_samples,1); Yp3 = zeros(N_samples,1);

Xp1_init = 100; Yp1_init = 0;  
Xv1 = 0.1; Yv1 = 0.1;           
Xp2_init = 20; Yp2_init = 5;
Xv2 = 0; Yv2 = -0.2;
Xp3_init = 50; Yp3_init = -5;
Xv3 = 0; Yv3 = 0.2;

Cov = [1 0 0 0; 0 1 0 0];
y_measure_v = zeros(2,N_samples*3);    
y_measure_r = zeros(2,N_samples*3);

measure_v = zeros(2,3);
measure_r = zeros(2,3);

Rv = diag([0.1 0.01]);
Rr = diag([0.01 0.1]);
index_v = 1; index_r = 1;   
vision_update = 1; radar_update = 1;
update_v = 0; update_r = 0;

saved_Xv = zeros(N_samples,3);  
saved_Yv = zeros(N_samples,3);

Phi_ov = [1 0 Ts 0; 0 1 0 Ts; 0 0 1 0; 0 0 0 1];
X1v_hat0 = [100 0 0.1 0.1]'; X2v_hat0 = [20 5 0 -0.2]'; X3v_hat0 = [50 -5 0 0.2]';
X1r_hat0 = [100 0 0.1 0.1]'; X2r_hat0 = [20 5 0 -0.2]'; X3r_hat0 = [50 -5 0 0.2]';

for k=1:Ts:N_samples
    
   
    if(k == 1)
        Xp1(k) = Xp1_init; Yp1(k) = Yp1_init;  
        Xp2(k) = Xp2_init; Yp2(k) = Yp2_init;
        Xp3(k) = Xp3_init; Yp3(k) = Yp3_init;
    else
        Xp1(k) = Xp1(k-1) + Xv1*Ts; Yp1(k) = Yp1(k-1) + Yv1*Ts;    
        Xp2(k) = Xp2(k-1) + Xv2*Ts; Yp2(k) = Yp2(k-1) + Yv2*Ts;
        Xp3(k) = Xp3(k-1) + Xv3*Ts; Yp3(k) = Yp3(k-1) + Yv3*Ts;
    end
    Vehicle_X1 = [ Xp1(k) Yp1(k) Xv1 Yv1 ]';   
    Vehicle_X2 = [ Xp2(k) Yp2(k) Xv2 Yv2 ]';
    Vehicle_X3 = [ Xp3(k) Yp3(k) Xv3 Yv3 ]';
    
    saved_Xv(k,:) = [Xv1 Xv2 Xv3];      
    saved_Yv(k,:) = [Yv1 Yv2 Yv3];
    
    
    if(rem(k,Rm_v)==0)  
        index_random = randperm(3);
        vision_update = vision_update + 1;
        
       
        measure_v(:,index_random(1)) = Cov * Vehicle_X1 + [sqrt(Rv(1,1))*randn(1) sqrt(Rv(2,2))*randn(1)]';
        measure_v(:,index_random(2)) = Cov * Vehicle_X2 + [sqrt(Rv(1,1))*randn(1) sqrt(Rv(2,2))*randn(1)]';
        measure_v(:,index_random(3)) = Cov * Vehicle_X3 + [sqrt(Rv(1,1))*randn(1) sqrt(Rv(2,2))*randn(1)]';
        
     
        for i=1:3
            mahalanobis_dis1(i) = abs(([measure_v(1,i); measure_v(2,i)] - [ Vehicle_X1(1); Vehicle_X1(2)])' * inv(Rv) * ([measure_v(1,1); measure_v(2,1)] - [ Vehicle_X1(1); Vehicle_X1(2)]));
            mahalanobis_dis2(i) = abs(([measure_v(1,i); measure_v(2,i)] - [ Vehicle_X2(1); Vehicle_X2(2)])' * inv(Rv) * ([measure_v(1,1); measure_v(2,1)] - [ Vehicle_X2(1); Vehicle_X2(2)]));
            mahalanobis_dis3(i) = abs(([measure_v(1,i); measure_v(2,i)] - [ Vehicle_X3(1); Vehicle_X3(2)])' * inv(Rv) * ([measure_v(1,1); measure_v(2,1)] - [ Vehicle_X3(1); Vehicle_X3(2)]));
        end
        min_v1 = min([mahalanobis_dis1(1) mahalanobis_dis1(2) mahalanobis_dis1(3)]);
        if(min_v1 == mahalanobis_dis1(1) )
            ymv_index1(:,vision_update) = measure_v(:,1);
        else if(min_v1 == mahalanobis_dis1(2) )
                ymv_index1(:,vision_update) = measure_v(:,2);
            else
                ymv_index1(:,vision_update) = measure_v(:,3);
            end
        end
        min_v2 = min([mahalanobis_dis2(1) mahalanobis_dis2(2) mahalanobis_dis2(3)]);
        if(min_v2 == mahalanobis_dis2(1) )
            ymv_index2(:,vision_update) = measure_v(:,1);
        else if(min_v2 == mahalanobis_dis2(2) )
                ymv_index2(:,vision_update) = measure_v(:,2);
            else
                ymv_index2(:,vision_update) = measure_v(:,3);
            end
        end
        min_v3 = min([mahalanobis_dis3(1) mahalanobis_dis3(2) mahalanobis_dis3(3)]);
        if(min_v3 == mahalanobis_dis3(1) )
            ymv_index3(:,vision_update) = measure_v(:,1);
        else if(min_v3 == mahalanobis_dis3(2) )
                ymv_index3(:,vision_update) = measure_v(:,2);
            else
                ymv_index3(:,vision_update) = measure_v(:,3);
            end
        end
    end
   
    if(rem(k,Rm_r)==0) 
        index_random = randperm(3);
        radar_update = radar_update + 1;
        
       
        measure_r(:,index_random(1)) = Cov * Vehicle_X1 + [sqrt(Rr(1,1))*randn(1) sqrt(Rr(2,2))*randn(1)]';
        measure_r(:,index_random(2)) = Cov * Vehicle_X2 + [sqrt(Rr(1,1))*randn(1) sqrt(Rr(2,2))*randn(1)]';
        measure_r(:,index_random(3)) = Cov * Vehicle_X3 + [sqrt(Rr(1,1))*randn(1) sqrt(Rr(2,2))*randn(1)]';
        
        
        for i=1:3
            mahalanobis_dis1(i) = abs(([measure_r(1,i); measure_r(2,i)] - [ Vehicle_X1(1); Vehicle_X1(2)])' * inv(Rr) * ([measure_r(1,1); measure_r(2,1)] - [ Vehicle_X1(1); Vehicle_X1(2)]));
            mahalanobis_dis2(i) = abs(([measure_r(1,i); measure_r(2,i)] - [ Vehicle_X2(1); Vehicle_X2(2)])' * inv(Rr) * ([measure_r(1,1); measure_r(2,1)] - [ Vehicle_X2(1); Vehicle_X2(2)]));
            mahalanobis_dis3(i) = abs(([measure_r(1,i); measure_r(2,i)] - [ Vehicle_X3(1); Vehicle_X3(2)])' * inv(Rr) * ([measure_r(1,1); measure_r(2,1)] - [ Vehicle_X3(1); Vehicle_X3(2)]));
        end
        min_r1 = min([mahalanobis_dis1(1) mahalanobis_dis1(2) mahalanobis_dis1(3)]);
        if(min_r1 == mahalanobis_dis1(1) )
            ymr_index1(:,radar_update) = measure_r(:,1);
        else if(min_r1 == mahalanobis_dis1(2) )
                ymr_index1(:,radar_update) = measure_r(:,2);
            else
                ymr_index1(:,radar_update) = measure_r(:,3);
            end
        end
        min_r2 = min([mahalanobis_dis2(1) mahalanobis_dis2(2) mahalanobis_dis2(3)]);
        if(min_r2 == mahalanobis_dis2(1) )
            ymr_index2(:,radar_update) = measure_r(:,1);
        else if(min_r2 == mahalanobis_dis2(2) )
                ymr_index2(:,radar_update) = measure_r(:,2);
            else
                ymr_index2(:,radar_update) = measure_r(:,3);
            end
        end
        min_r3 = min([mahalanobis_dis3(1) mahalanobis_dis3(2) mahalanobis_dis3(3)]);
        if(min_r3 == mahalanobis_dis3(1) )
            ymr_index3(:,radar_update) = measure_r(:,1);
        else if(min_r3 == mahalanobis_dis3(2) )
                ymr_index3(:,radar_update) = measure_r(:,2);
            else
                ymr_index3(:,radar_update) = measure_r(:,3);
            end
        end
    end
   
    if(k == 1)
        X1v_bar(:,k) =  X1v_hat0;   
        X2v_bar(:,k) =  X2v_hat0 ;
        X3v_bar(:,k) =  X3v_hat0 ;
        
        X1r_bar(:,k) =  X1r_hat0;   
        X2r_bar(:,k) =  X2r_hat0;
        X3r_bar(:,k) =  X3r_hat0;
    else
        X1v_bar(:,k) = Phi_ov * X1v_hat(:,k-1) ;     
        X2v_bar(:,k) = Phi_ov * X2v_hat(:,k-1);
        X3v_bar(:,k) = Phi_ov * X3v_hat(:,k-1);
        
        X1r_bar(:,k) = Phi_ov * X1r_hat(:,k-1);    
        X2r_bar(:,k) = Phi_ov * X2r_hat(:,k-1);
        X3r_bar(:,k) = Phi_ov * X3r_hat(:,k-1);
    end
    
    
    AA = [1 0 1 0; 0 1 0 1; 0 0 1 0; 0 0 0 1];
    BB = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    Pv = [exp(-0.02) exp(-0.03) exp(-0.04) exp(-0.05)]; 
   
    Lv = place(AA,BB,Pv)*[1 0;0 1; 0 0; 0 0];           
    Pr = [exp(-0.03) exp(-0.02) exp(-0.04) exp(-0.05)];
    Lr = place(AA,BB,Pr)*[1 0;0 1; 0 0; 0 0];           
    
    if(rem(k,Rm_v)==0 || k==1)  
        update_v = update_v+1;
        X1v_bar_updata(:,update_v) = X1v_bar(:,k);
        X2v_bar_updata(:,update_v) = X2v_bar(:,k);
        X3v_bar_updata(:,update_v) = X3v_bar(:,k);
    end
    if(rem(k,Rm_r)==0 || k==1)  
        update_r = update_r+1;
        X1r_bar_updata(:,update_r) = X1r_bar(:,k);
        X2r_bar_updata(:,update_r) = X2r_bar(:,k);
        X3r_bar_updata(:,update_r) = X3r_bar(:,k);
    end
    
    if( k == 1)
        ymv_index1(:,k) = [100 0]'; ymv_index2(:,k) = [20 5]'; ymv_index3(:,k) = [50 -5]';
        ymr_index1(:,k) = [100 0]'; ymr_index2(:,k) = [20 5]'; ymr_index3(:,k) = [50 -5]';
    end
    
   
    X1v_hat(:,k) = X1v_bar(:,k) + Lv*(ymv_index1(:,vision_update) - Cov*X1v_bar_updata(:,update_v));    
    X2v_hat(:,k) = X2v_bar(:,k) + Lv*(ymv_index2(:,vision_update) - Cov*X2v_bar_updata(:,update_v));
    X3v_hat(:,k) = X3v_bar(:,k) + Lv*(ymv_index3(:,vision_update) - Cov*X3v_bar_updata(:,update_v));
    
    X1r_hat(:,k) = X1r_bar(:,k) + Lr*(ymr_index1(:,radar_update) - Cov*X1r_bar_updata(:,update_r));     
    X2r_hat(:,k) = X2r_bar(:,k) + Lr*(ymr_index2(:,radar_update) - Cov*X2r_bar_updata(:,update_r));
    X3r_hat(:,k) = X3r_bar(:,k) + Lr*(ymr_index3(:,radar_update) - Cov*X3r_bar_updata(:,update_r));
   
    error1v(k,:) =  sqrt((Xp1(k)-X1v_hat(1,k))^2 + (Yp1(k)-X1v_hat(2,k))^2);   
    error2v(k,:) =  sqrt((Xp2(k)-X2v_hat(1,k))^2 + (Yp2(k)-X2v_hat(2,k))^2);
    error3v(k,:) =  sqrt((Xp3(k)-X3v_hat(1,k))^2 + (Yp3(k)-X3v_hat(2,k))^2);
    
    error1r(k,:) =  sqrt((Xp1(k)-X1r_hat(1,k))^2 + (Yp1(k)-X1r_hat(2,k))^2);    
    error2r(k,:) =  sqrt((Xp2(k)-X2r_hat(1,k))^2 + (Yp2(k)-X2r_hat(2,k))^2);
    error3r(k,:) =  sqrt((Xp3(k)-X3r_hat(1,k))^2 + (Yp3(k)-X3r_hat(2,k))^2);
    
    
    
   
    
     vehicle1_mahalanobis_dis1(k,1) =  sqrt((X1v_hat(1,k)-X1r_hat(1,k))^2 + (X1v_hat(2,k)-X1r_hat(2,k))^2);
     vehicle1_mahalanobis_dis2(k,1) =  sqrt((X1v_hat(1,k)-X2r_hat(1,k))^2 + (X1v_hat(2,k)-X2r_hat(2,k))^2);
     vehicle1_mahalanobis_dis3(k,1) =  sqrt((X1v_hat(1,k)-X3r_hat(1,k))^2 + (X1v_hat(2,k)-X3r_hat(2,k))^2);
     min_dis1 = min([vehicle1_mahalanobis_dis1(k,1) vehicle1_mahalanobis_dis1(k,1) vehicle1_mahalanobis_dis1(k,1)]);
     if(min_dis1 == vehicle1_mahalanobis_dis1(k,1))
         index_v1 = 1;  index_r1 = 1;
         X1_hat(1,k) = X1v_hat(1,k)*0.2 + X1r_hat(1,k)*0.8;
         X1_hat(2,k) = X1v_hat(2,k)*0.8 + X1r_hat(2,k)*0.2;        
     else if(min_dis1 == vehicle1_mahalanobis_dis2(k,1))
             index_v1 = 1;  index_r1 = 2;
             X1_hat(1,k) = X1v_hat(1,k)*0.2 + X2r_hat(1,k)*0.8;
             X1_hat(2,k) = X1v_hat(2,k)*0.8 + X2r_hat(2,k)*0.2;     
         else
             index_v1 = 1;  index_r1 = 3;
             X1_hat(1,k) = X1v_hat(1,k)*0.2 + X3r_hat(1,k)*0.8;
             X1_hat(2,k) = X1v_hat(2,k)*0.8 + X3r_hat(2,k)*0.2;     
         end
     end
     vehicle2_mahalanobis_dis1(k,1) =  sqrt((X2v_hat(1,k)-X1r_hat(1,k))^2 + (X2v_hat(2,k)-X1r_hat(2,k))^2);
     vehicle2_mahalanobis_dis2(k,1) =  sqrt((X2v_hat(1,k)-X2r_hat(1,k))^2 + (X2v_hat(2,k)-X2r_hat(2,k))^2);
     vehicle2_mahalanobis_dis3(k,1) =  sqrt((X2v_hat(1,k)-X3r_hat(1,k))^2 + (X2v_hat(2,k)-X3r_hat(2,k))^2);
     min_dis2 = min([vehicle2_mahalanobis_dis1(k,1) vehicle2_mahalanobis_dis2(k,1) vehicle2_mahalanobis_dis3(k,1)]);
     if(min_dis2 == vehicle2_mahalanobis_dis1(k,1))
         index_v2 = 2;  index_r2 = 1;
         X2_hat(1,k) = X2v_hat(1,k)*0.2 + X1r_hat(1,k)*0.8; 
         X2_hat(2,k) = X2v_hat(2,k)*0.8 + X1r_hat(2,k)*0.2;
     else if(min_dis2 == vehicle2_mahalanobis_dis2(k,1))
             index_v2 = 2;  index_r2 = 2;
             X2_hat(1,k) = X2v_hat(1,k)*0.2 + X2r_hat(1,k)*0.8; 
             X2_hat(2,k) = X2v_hat(2,k)*0.8 + X2r_hat(2,k)*0.2;
         else
             index_v2 = 2;  index_r2 = 3;
             X2_hat(1,k) = X2v_hat(1,k)*0.2 + X3r_hat(1,k)*0.8; 
             X2_hat(2,k) = X2v_hat(2,k)*0.8 + X3r_hat(2,k)*0.2;
         end
     end
     vehicle3_mahalanobis_dis1(k,1) =  sqrt((X3v_hat(1,k)-X1r_hat(1,k))^2 + (X3v_hat(2,k)-X1r_hat(2,k))^2);
     vehicle3_mahalanobis_dis2(k,1) =  sqrt((X3v_hat(1,k)-X2r_hat(1,k))^2 + (X3v_hat(2,k)-X2r_hat(2,k))^2);
     vehicle3_mahalanobis_dis3(k,1) =  sqrt((X3v_hat(1,k)-X3r_hat(1,k))^2 + (X3v_hat(2,k)-X3r_hat(2,k))^2);
     min_dis3 = min([vehicle3_mahalanobis_dis1(k,1) vehicle3_mahalanobis_dis2(k,1) vehicle3_mahalanobis_dis3(k,1)]);
     if(min_dis3 == vehicle3_mahalanobis_dis1(k,1))
         index_v3 = 3;  index_r3 = 1;
         X3_hat(1,k) = X3v_hat(1,k)*0.2 + X1r_hat(1,k)*0.8; 
         X3_hat(2,k) = X3v_hat(2,k)*0.8 + X1r_hat(2,k)*0.2;
     else if(min_dis3 == vehicle3_mahalanobis_dis2(k,1))
             index_v3 = 3;  index_r3 = 2;
             X3_hat(1,k) = X3v_hat(1,k)*0.2 + X2r_hat(1,k)*0.8; 
             X3_hat(2,k) = X3v_hat(2,k)*0.8 + X2r_hat(2,k)*0.2;
         else
             index_v3 = 3;  index_r3 = 3;
             X3_hat(1,k) = X3v_hat(1,k)*0.2 + X3r_hat(1,k)*0.8; 
             X3_hat(2,k) = X3v_hat(2,k)*0.8 + X3r_hat(2,k)*0.2;
         end
     end
     
    
     error1(k,:) =  sqrt((Xp1(k)-X1_hat(1,k))^2 + (Yp1(k)-X1_hat(2,k))^2);
     error2(k,:) =  sqrt((Xp2(k)-X2_hat(1,k))^2 + (Yp2(k)-X2_hat(2,k))^2);
     error3(k,:) =  sqrt((Xp3(k)-X3_hat(1,k))^2 + (Yp3(k)-X3_hat(2,k))^2);  
    
end

xmin = -16; xmax = 16; ymin = 0; ymax = 120;
Yp = [Yp1;Yp2;Yp3]; Xp = [Xp1;Xp2;Xp3];
fov_v1 = [ 0 0;10 17.32]; fov_v2 = [ 0 0;-10 17.32];
fov_r1 = [ 0 0;10 56.71]; fov_r2 = [ 0 0;-10 56.71];

figure(1)   
plot(Yp1,Xp1,'bo'); hold on; plot(Yp2,Xp2,'ro'); plot(Yp3,Xp3,'co'); legend('Object 1','Object 2','Object 3');
xlim([xmin,xmax]); ylim([ymin,ymax]);  set(gca, 'XDir', 'reverse'); xlabel({'← Position Y '}); ylabel({'Position X →'}); hold off
title('Object True Position');

figure(8)   
plot(ymv_index1(2,:),ymv_index1(1,:),'r^'); hold on;plot(ymr_index1(2,:),ymr_index1(1,:),'b*');
plot(ymv_index2(2,:),ymv_index2(1,:),'r^'); plot(ymv_index3(2,:),ymv_index3(1,:),'r^');
plot(ymr_index2(2,:),ymr_index2(1,:),'b*'); plot(ymr_index3(2,:),ymr_index3(1,:),'b*');
legend('Measure Vision','Measure Radar'); title('vision, radar 측정값');
xlim([xmin,xmax]); ylim([0,ymax]);  set(gca, 'XDir', 'reverse'); xlabel({'← Position Y '}); ylabel({'Position X →'}); hold off

figure(9)   
plot(Yp,Xp,'ko'); hold on; 
plot(ymv_index1(2,:),ymv_index1(1,:),'r^'); hold on; plot(ymv_index2(2,:),ymv_index2(1,:),'r^'); plot(ymv_index3(2,:),ymv_index3(1,:),'r^');
legend('Vehicles','Measure Vision'); title('vision 측정값 및 차량 위치');
xlim([xmin,xmax]); ylim([ymin,ymax]);  set(gca, 'XDir', 'reverse'); xlabel({'← Position Y '}); ylabel({'Position X →'}); hold off

figure(10) 
plot(Yp,Xp,'ko'); hold on; 
plot(ymr_index1(2,:),ymr_index1(1,:),'b*'); hold on; plot(ymr_index2(2,:),ymr_index2(1,:),'b*'); plot(ymr_index3(2,:),ymr_index3(1,:),'b*');
legend('Vehicles','Measure Radar'); title('radar 측정값 및 차량 위치');
xlim([xmin,xmax]); ylim([ymin,ymax]);  set(gca, 'XDir', 'reverse'); xlabel({'← Position Y '}); ylabel({'Position X →'}); hold off

figure(11)   
plot(ymv_index1(2,:),ymv_index1(1,:),'r^'); hold on; plot(ymr_index1(2,:),ymr_index1(1,:),'b*');
plot(ymv_index2(2,:),ymv_index2(1,:),'r^'); plot(ymv_index3(2,:),ymv_index3(1,:),'r^');
plot(ymr_index2(2,:),ymr_index2(1,:),'b*'); plot(ymr_index3(2,:),ymr_index3(1,:),'b*');
legend('Measure Vision','Measure Radar'); title('vision, radar 측정값');
xlim([xmin,xmax]); ylim([ymin,ymax]);  set(gca, 'XDir', 'reverse'); xlabel({'← Position Y '}); ylabel({'Position X →'}); hold off


figure(12)
plot1 = subplot(2,1,1); plot2 = subplot(2,1,2);
plot(plot1,saved_Xv,'DisplayName','saved_Xv');  plot(plot2,saved_Yv,'DisplayName','saved_Xv');
legend(plot1,'vehicle 1','vehecle 2','vehicle 3');  legend(plot2,'vehicle 1','vehecle 2','vehicle 3');
xlim(plot1,[0,N_samples]); ylim(plot1,[-0.2,0.2]); xlabel(plot1,{' 시간 t '}); ylabel(plot1,{'X축 속도'});
xlim([0,N_samples]); ylim([-0.3,0.3]); xlabel(plot2,{' 시간 t '}); ylabel(plot2,{'Y축 속도'}); hold off
title(plot1,'차량 종방향 속도'); title(plot2,'차량 횡방향 속도');


figure(14)  
plot(X1v_hat(2,:),X1v_hat(1,:),'b^'); hold on; plot(X2v_hat(2,:),X2v_hat(1,:),'r^'); plot(X3v_hat(2,:),X3v_hat(1,:),'c^');
legend('v1 hat','v2 hat','v3 hat'); title('vision 추정값');
xlim([xmin,xmax]); ylim([ymin,ymax]);  set(gca, 'XDir', 'reverse'); xlabel({'← Position Y '}); ylabel({'Position X →'}); hold off

figure(15)  
plot(X1v_bar(2,:),X1v_bar(1,:),'b+'); hold on; plot(X2v_bar(2,:),X2v_bar(1,:),'r+'); plot(X3v_bar(2,:),X3v_bar(1,:),'c+');
legend('v1 har','v2 bar','v3 bar'); title('vision 예측값');
xlim([xmin,xmax]); ylim([ymin,ymax]);  set(gca, 'XDir', 'reverse'); xlabel({'← Position Y '}); ylabel({'Position X →'}); hold off

figure(18)   
plot(Yp1,Xp1,'ko'); hold on; plot(X1v_hat(2,:),X1v_hat(1,:),'r^');
plot(X2v_hat(2,:),X2v_hat(1,:),'r^'); hold on;  plot(Yp2,Xp2,'ko'); plot(X3v_hat(2,:),X3v_hat(1,:),'r^'); hold on;  plot(Yp3,Xp3,'ko');
legend('vehicle true','vision estimation'); title('vision 추정값, 실제 차량 위치');
xlim([xmin,xmax]); ylim([ymin,ymax]);  set(gca, 'XDir', 'reverse'); xlabel({'← Position Y '}); ylabel({'Position X →'}); hold off

figure(19) 
plot(Yp1,Xp1,'ko'); hold on; plot(X1r_hat(2,:),X1r_hat(1,:),'b*');
plot(X2r_hat(2,:),X2r_hat(1,:),'b*'); hold on;  plot(Yp2,Xp2,'ko'); plot(X3r_hat(2,:),X3r_hat(1,:),'b*'); hold on;  plot(Yp3,Xp3,'ko');
legend('vehicle true','radar estimation'); title('radar 추정값, 실제 차량 위치');
xlim([xmin,xmax]); ylim([ymin,ymax]);  set(gca, 'XDir', 'reverse'); xlabel({'← Position Y '}); ylabel({'Position X →'}); hold off

plot(X1_hat(2,:),X1_hat(1,:),'rs'); hold on;  plot(X2_hat(2,:),X2_hat(1,:),'bs');  plot(X3_hat(2,:),X3_hat(1,:),'cs');
legend('sensor-fusion vehicle 1','sensor-fusion vehicle 2','sensor-fusion vehicle 3');
xlim([xmin,xmax]); ylim([0,ymax]);  set(gca, 'XDir', 'reverse'); xlabel({'← Position Y '}); ylabel({'Position X →'}); hold off
title('sensor fusion 차량 위치 결과');

figure(21) 
plot(X1_hat(2,:),X1_hat(1,:),'c+'); hold on; plot(Yp,Xp,'ko'); hold on; 
plot(X2_hat(2,:),X2_hat(1,:),'c+');  plot(X3_hat(2,:),X3_hat(1,:),'c+');
legend('sensor-fusion vehicle','vehicle true'); title('sensor-fusion과 실제 위치 비교');
xlim([xmin,xmax]); ylim([ymin,ymax]);  set(gca, 'XDir', 'reverse'); xlabel({'← Position Y '}); ylabel({'Position X →'}); hold off

figure(22)  
plot(error1v,'r-'); hold on; plot(error2v,'b-'); hold on; plot(error3v,'c-'); hold on;
legend('vehicle 1','vehicle 2','vehicle 3'); title('vision 추정값과 실제 위치 오차');
xlim([0,N_samples]); ylim([0,0.15]); xlabel({'시간 t'}); ylabel({'error'}); hold off

figure(23)  
plot(error1r,'r-'); hold on; plot(error2r,'b-'); hold on; plot(error3r,'c-'); hold on;
legend('vehicle 1','vehicle 2','vehicle 3'); title('radar 추정값과 실제 위치 오차');
xlim([0,N_samples]); ylim([0,0.15]); xlabel({'시간 t'}); ylabel({'error'}); hold off

figure(24)  
plot(error1,'r-'); hold on; plot(error2,'b-'); hold on; plot(error3,'c-'); hold on;
legend('vehicle 1','vehicle 2','vehicle 3'); title('sensor-fusion과 실제 위치 오차');
xlim([0,N_samples]); ylim([0,0.15]); xlabel({'시간 t'}); ylabel({'error'}); hold off
