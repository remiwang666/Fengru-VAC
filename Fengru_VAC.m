clear all
close all

%il faut faire CTRL C pour arreter la simulation
%il n'y a pas d'intelligence  ; si 2 voitures se retrouvent face à face elles s'arretent
starter=0;
% Construct a drivingScenario object.
Ts = 1; %sampling time
scenario = drivingScenario('SampleTime', Ts);
scale_scenario=20;
% Add all road segments
L=800/scale_scenario;    %dimension d'un segment
for i=0:5
    for j=0:5 
        if j<5
            roadCenters = [i*L j*L 0 ;i*L (j+1)*L 0];
            laneSpecification = lanespec(1);
            road(scenario, roadCenters, 'Lanes', laneSpecification);
        end
        if i<5
            roadCenters = [i*L j*L 0 ;(i+1)*L j*L 0];
            laneSpecification = lanespec(1);
            road(scenario, roadCenters, 'Lanes', laneSpecification);
        end
    end
end

Nb_carA=10;Nb_carB=15;Nb_car=Nb_carA+Nb_carB;
interval=10;

for i=1:Nb_carB
    car(i) = vehicle(scenario, 'ClassID', 'B', 'Position', [0 0 0]);
    speed(i)=5*20/scale_scenario;
    point_livraison(i,:)= [5*L 5*L 0];  %point de livraison du coli
    last_pos_list(i,:)= [5*L 5*L 0];  
    T0(i)=interval*i;  %date de départ de la voiture
    Livraison_OK(i)=0;      %1 = coli livré
    stop(i)=0;     %si stop=1 la voiture est à l'arret
    fin(i)=0;
end   

for i=(1+Nb_carB):Nb_carB+Nb_carA 
    car(i) = vehicle(scenario, 'ClassID', 'A', 'Position', [0 5*L 0]);
    speed(i)=3*20/scale_scenario;
    point_livraison(i,:)= [5*L 0 0];  %point de livraison du coli    
    last_pos_list(i,:)= [5*L 0 0];
    T0(i)=interval*(i-Nb_carB);  %date de départ de la voiture
    Livraison_OK(i)=0;      %1 = coli livré
    stop(i)=0;     %si stop=1 la voiture est à l'arret
    fin(i)=0;
end

% long time go back
count_stay=zeros(1,Nb_car);
count_seuil=40;
count_add=5;

% Set all stations
rayon=400/scale_scenario*2.5;
station_taux={};
station_taux_max=[];
station_position={};
station_position_max={};
for i=1:6
    for j=1:6
        station_position{i,j}=[L*(i-1);L*(j-1)];
    end
end

% relation CSMA taux
Tf=4.8*10e-6;
lambda=2;
q=1-exp(-lambda*Tf);
m=2;
W=5;
N=20;
init_pc=0.5;
init_pin=0.5;
maxnum=500;
num2taux=zeros(1,maxnum);
for i=2:maxnum
    num2taux(i)=CSMA(i,q,m,W,init_pc,init_pin,N);
end

% network nodes nomination and construct the directed graph
last_intersection_pos_dict={};
destination_dict={};
nodes_index2pos={};
nodes_indexpair2pos={};
nodes_index2indexpair={};
nodes_indexpair2index=[];
from_list=[];
to_list=[];
weight_list=ones(1,60*2);

for i=1:6
    for j=1:6
        nodes_indexpair2index(i,j)=(j-1)*6+i;
    end
end

for k=1:36
    nodes_index2pos{k}=[mod(k-1,6)*L;floor((k-1)/6)*L];
    nodes_index2indexpair{k}=[mod(k-1,6)+1 floor((k-1)/6)+1];
end

for i=1:6
    for j=1:6
        if i==1
            from_list=[from_list nodes_indexpair2index(i+1,j)];
            to_list=[to_list nodes_indexpair2index(i,j)];
        elseif i==6
            from_list=[from_list nodes_indexpair2index(i-1,j)];
            to_list=[to_list nodes_indexpair2index(i,j)];
        else
            from_list=[from_list nodes_indexpair2index(i+1,j)];
            to_list=[to_list nodes_indexpair2index(i,j)];
            from_list=[from_list nodes_indexpair2index(i-1,j)];
            to_list=[to_list nodes_indexpair2index(i,j)];
        end
        
        if j==1
            from_list=[from_list nodes_indexpair2index(i,j+1)];
            to_list=[to_list nodes_indexpair2index(i,j)];
        elseif j==6
            from_list=[from_list nodes_indexpair2index(i,j-1)];
            to_list=[to_list nodes_indexpair2index(i,j)];
        else
            from_list=[from_list nodes_indexpair2index(i,j+1)];
            to_list=[to_list nodes_indexpair2index(i,j)];
            from_list=[from_list nodes_indexpair2index(i,j-1)];
            to_list=[to_list nodes_indexpair2index(i,j)];
        end
    end
end
basic_graph=sparse(from_list,to_list,weight_list);

% unisense routes
% 0: really unisense, very high weight, cannot turn back
% 1: not really unisense, relatively high weight, can turn back
uni_route(1,:)=[6 12 0];
uni_route=[uni_route;[5 6 0]];
uni_route=[uni_route;[36 30 0]];
uni_route=[uni_route;[35 36 0]];
uni_route=[uni_route;[4 5 0]];
uni_route=[uni_route;[34 35 0]];

uni_route=[uni_route;[25 31 1]];
uni_route=[uni_route;[32 31 0]];
uni_route=[uni_route;[7 1 1]];
uni_route=[uni_route;[2 1 0]];

uni_route=[uni_route;[33 32 1]];
uni_route=[uni_route;[19 25 1]];
uni_route=[uni_route;[13 7 1]];
uni_route=[uni_route;[3 2 1]];

uni_route=[uni_route;[8 2 1]];
uni_route=[uni_route;[8 7 1]];
uni_route=[uni_route;[26 32 1]];
uni_route=[uni_route;[26 25 1]];

% give priority to cars that have finished their delivery and are near to the departure points
point_dest=point_livraison;
station_rayon=3.1*car(1).Length;

plot(scenario)  
t=0; % to record lossing rate during the simulation

while advance(scenario)
    % check if at intersections
    [at_intersection_list,last_intersection_pos_dict]=at_intersection(car,L,last_intersection_pos_dict);
    
    % pcc
    
    % give weights to the graph based on the cars on roads but not at intersections
    graph_temps=basic_graph;
    
    blocage_weight=10;
    spread_weight=1;

    for j=1:Nb_car
        if at_intersection_list(j)==0
            [from_point,to_point]=find_car_line(car(j),L,nodes_indexpair2index);
            [from_point_list,to_point_list]=spread_influence(from_point,to_point,nodes_index2indexpair,nodes_indexpair2index);
            for i=1:width(from_point_list)
                graph_temps(to_point_list(i),from_point_list(i))=graph_temps(to_point_list(i),from_point_list(i))+spread_weight;
                graph_temps(from_point_list(i),to_point_list(i))=graph_temps(from_point_list(i),to_point_list(i))+spread_weight/2;
            end
            graph_temps(to_point,from_point)=graph_temps(to_point,from_point)+blocage_weight;
        end
    end
    
    % record car positions for debuging
    positions_x=[];
    positions_y=[];
    
    for j=1:Nb_car
        positions_x=[positions_x car(j).Position(1)];
        positions_y=[positions_y car(j).Position(2)];
    end
    
    % special treatment for delivery points and departure points (unisense roads)
    
    for i=1:height(uni_route)
        points=uni_route(i,:);
        if points(3)==0
            graph_temps(points(1),points(2))=1000;
        elseif points(3)==1
            graph_temps(points(1),points(2))=100;
        end
        
    end
    
    % pcc for point cars
    for j=1:Nb_car
        if at_intersection_list(j)==1 && fin(j)==0
            start_index_x=floor(car(j).Position(1)/L)+1;
            start_index_y=floor(car(j).Position(2)/L)+1;
            start_point_index=nodes_indexpair2index(start_index_x,start_index_y);
            
            final_point=point_livraison(j,1:2)';
            final_index_x=floor(final_point(1)/L)+1;
            final_index_y=floor(final_point(2)/L)+1;
            final_point_index=nodes_indexpair2index(final_index_x,final_index_y);
            
            [dist,path,pred] = graphshortestpath(graph_temps,start_point_index,final_point_index);
            if width(path)>1
                dest_index=path(2);
                dest_pos=nodes_index2pos(dest_index);
                point_dest(j,:)=[cell2mat(dest_pos)' 0];
            end
        end
    end
    
    % pcc for line cars when being blocked for long time
    for j=1:Nb_car
        if at_intersection_list(j)==0 && fin(j)==0
            if stop(j)==1
                count(j)=count(j)+randi(count_add)*speed(j);
                if count(j)>count_seuil && (randi(15)<(speed(j)*(Livraison_OK(j)+1))) % quicker cars have higher probability to turn back
                    count(j)=0;
                    [from_point,to_point]=find_car_line(car(j),L,nodes_indexpair2index);
                    if isnan(to_point)==0
                        
                        % forbidden cars to turn back on unisense roads
                        check_unisirection=0; 
                        for i=1:height(uni_route)
                            points=uni_route(i,:);
                            if from_point==points(2) && to_point==points(1) && points(3)==0
                                check_unisirection=1;
                                break
                            end
                        end
                        if check_unisirection==1
                            continue
                        end
                        
                        final_point=point_livraison(j,1:2)';
                        final_index_x=floor(final_point(1)/L)+1;
                        final_index_y=floor(final_point(2)/L)+1;
                        final_point_index=nodes_indexpair2index(final_index_x,final_index_y);
                        
                        % compare the weight of turning back and that of continuing being blocked
                        [dist_to,~,~] = graphshortestpath(graph_temps,to_point,final_point_index);
                        [dist_from,~,~] = graphshortestpath(graph_temps,from_point,final_point_index);

                        if dist_from<graph_temps(from_point,to_point)+dist_to  
                            car(j).Yaw=car(j).Yaw+180;
                            point_dest(j,:)=[cell2mat(nodes_index2pos(from_point));0]';
                        end
                    end
                end
            else
                count(j)=0; % reset the accumulation to zero
            end
        end
    end
    
    for j=1:Nb_car
        %detection of collisions
        flag_stop=zeros(1,Nb_car);
        flag_obstacle_front=zeros(1,Nb_car);
        flag_obstacle_left=zeros(1,Nb_car);
        flag_obstacle_right=zeros(1,Nb_car);
        for i=1:Nb_car
            if (i~=j) && (scenario.SimulationTime>T0(i)) && (scenario.SimulationTime>T0(j)) && (fin(i)==0) %on ne regarde que les voiture deja partie livrer
                scenario.SimulationTime;
                [zoneX,zoneY,flag_stop(i),flag_obstacle_front(i),flag_obstacle_left(i),flag_obstacle_right(i)] = distance(car(j),car(i));
            end
        end
        
        if max(flag_stop)==1 % if an obstacle is found, stop at first to be prudent
            stop(j)=1;
            
            if max(flag_obstacle_right)==0 % if have the priority of right, do not need to stop
                stop(j)=0;
            end
            
            if max(flag_obstacle_front)==1 
                for k=1:Nb_car
                    [obs_from_point,obs_to_point]=find_car_line(car(k),L,nodes_indexpair2index);
                    [ego_from_point,ego_to_point]=find_car_line(car(j),L,nodes_indexpair2index);
                    if at_intersection_list(k)==0 % if another car is at the interscetion, keep waiting
                        % if on the same road and the same direction, stop
                        if obs_to_point==ego_to_point && obs_from_point==ego_from_point 
                            stop(j)=1;
                            break
                        % if on the same road and the opposite direction, stop
                        elseif obs_to_point==ego_from_point && obs_from_point==ego_to_point 
                            stop(j)=1;
                            break
                        % if two cars are on different roads and the opposite direction, give priority to +x and +y if no car is on the left or right
                        elseif obs_to_point==ego_to_point && obs_from_point~=ego_from_point && min(max(flag_obstacle_left),max(flag_obstacle_right))==0 && mod(car(j).Yaw/90,4)<1.1
                            stop(j)=0;
                        end
                    end
                end

            end
        else stop(j)=0;
        end  %la voiture s'arrete si au moins une autre voiture est devant
    end
    
    % give priority to cars that have finished their delivery and are near to the departure points
    for j=1:Nb_car
        if Livraison_OK(j)==1 && norm(car(j).Position-point_livraison)<station_rayon
            stop(j)=0;
        end
    end

    for j=1:Nb_car
        if (scenario.SimulationTime>T0(j)) 
            [next_position, next_Yaw] = motion(car(j),point_dest(j,:),speed(j),Ts,stop(j));
            car(j).Position=next_position;
            car(j).Yaw=next_Yaw;
        end
        %verification si point de livraison atteind ; si oui, la voiture repart au point de depart
        if car(j).Position==point_livraison(j,:)
            if Livraison_OK(j)==0 
                Livraison_OK(j)=1;
                if j<=Nb_carB
                    point_livraison(j,:)=0;
                else
                    point_livraison(j,:)=[0 5*L 0];
                end
            else
                fin(j)=1;   %la voiture est revenue à son point de départ
            end
        end
    end
    
	updatePlots(scenario);
    
    % record loss rate during the simulation
    t=t+1;
    max_taux=0;
    max_i=1;
    max_j=1;
    for i=1:6
        for j=1:6
            num_car=count_car(station_position{i,j},car,rayon,T0,t,fin);
            if num_car>0
                station_taux{i,j,t}=num2taux(num_car);
            else
                station_taux{i,j,t}=0;
            end
            if station_taux{i,j,t}>max_taux
                max_taux=station_taux{i,j,t};
                max_i=i;
                max_j=j;
            end
        end
    end
    
    station_taux_max(t)=max_taux;
    station_position_max{t}=station_position{max_i,max_j};
    
    
    % at first, slow down the update of scenario to have enough time to enlarge the image window 
    if starter==0
        pause(2);
        starter=starter+1;
    else
        pause(0.04); 
    end
     %pour ralentir la simulation
end

% after the simulation, run these codes to show the loss rate over time
% time_list=1:width(station_taux_max);semilogy(time_list,station_taux_max);grid on;
% xlabel('Time (s)'); 
% ylabel('Loss probability'); 

function [next_Position next_Yaw] = motion(car,point_livraison,v,Ts,stop)
        next_Position=car.Position;
        if stop==1
            v=0;
        end
        
        if car.Position(1)~= point_livraison(1)
            if abs(point_livraison(1)-car.Position(1))>v*Ts
                next_Position(1)=car.Position(1)+v*Ts*sign(point_livraison(1)-car.Position(1));
            else
                next_Position(1)=point_livraison(1);
            end
            if (point_livraison(1)-car.Position(1))>0 next_Yaw = 0;else next_Yaw = 180;end
        elseif car.Position(2)~= point_livraison(2)
            if abs(point_livraison(2)-car.Position(2))>v*Ts
                next_Position(2)=car.Position(2)+v*Ts*sign(point_livraison(2)-car.Position(2));
            else
                next_Position(2)=point_livraison(2);
            end
            if (point_livraison(2)-car.Position(2))>0 next_Yaw = 90;else next_Yaw = -90;end
        else
            next_Position=car.Position;
            next_Yaw=car.Yaw;
        end
end

function [zoneX,zoneY,flag_stop,obstacle_front,obstacle_left,obstacle_right] = distance(vehicle,vehicle_obstacle)
obstacle_front=0;
obstacle_left=0;
obstacle_right=0;
%zone devant le vehicule
if abs(rem(vehicle.Yaw,180))<2
    zoneX=sort(real([vehicle.Position(1) vehicle.Position(1)+exp(1j*pi*vehicle.Yaw/180)*2.5*vehicle.Length]));
    zoneY=sort(real([vehicle.Position(2)-vehicle.Length*1.5 vehicle.Position(2)+vehicle.Length*1.5]));
else
    zoneX=sort(real([vehicle.Position(1)-vehicle.Length*1.5 vehicle.Position(1)+vehicle.Length*1.5]));
    zoneY=sort(real([vehicle.Position(2) vehicle.Position(2)+exp(1j*pi*(vehicle.Yaw-90)/180)*2.5*vehicle.Length]));  
end  

%detection si obstacle dans la zone and find where the obstale is from the point of view of the ego car
if (vehicle_obstacle.Position(1)>zoneX(1)) && (vehicle_obstacle.Position(1)<zoneX(2)) && ...
   (vehicle_obstacle.Position(2)>zoneY(1)) && (vehicle_obstacle.Position(2)<zoneY(2))
     flag_stop =1;
     if round(mod(vehicle.Yaw-vehicle_obstacle.Yaw,180))==0
         obstacle_front=1;
         obstacle_left=0;
         obstacle_right=0;
     else
         obstacle_front=0;
         if round(mod((vehicle.Yaw-vehicle_obstacle.Yaw)/90,4))==1
             obstacle_left=1;
             obstacle_right=0;
         else
             obstacle_left=0;
             obstacle_right=1;
         end
         
     end
else
     flag_stop =0;
end
end

% check the cars at intersections
% record the last intersections the cars passed through
function [at_intersection_list,last_intersection_pos_dict] = at_intersection(car,L,last_intersection_pos_dict)
at_intersection_list=zeros(1,width(car));
for k=1:width(car)
    car_pos=car(k).Position(1:2)';
    checkOK=0;
    for i=1:6
        for j=1:6
            if car_pos(1)==(i-1)*L && car_pos(2)==(j-1)*L 
                checkOK=1;
                at_intersection_list(k)=1;
            end
            if checkOK==1
                break
            end
        end
        if checkOK==1
            break
        end
    end
    if checkOK==1
        last_intersection_pos_dict{k}=[(i-1)*L;(j-1)*L];
    end
end
end

% find the directed road on which the car is
function [from_point,to_point]=find_car_line(car,L,nodes_indexpair2index)
    car_pos_x=car.Position(1);
    car_pos_y=car.Position(2);
    findOK=0;
    for i=1:6
        if car_pos_x==(i-1)*L
            for j=1:5
                if (car_pos_y>(j-1)*L) && (car_pos_y<j*L)
                    angle=car.Yaw;
                    findOK=1;
                    if mod(round(angle/90),4)==1
                        from_point_x=i;
                        from_point_y=j;
                        to_point_x=i;
                        to_point_y=j+1;
                    elseif mod(round(angle/90),4)==3
                        from_point_x=i;
                        from_point_y=j+1;
                        to_point_x=i;
                        to_point_y=j;
                    end
                end
                if findOK==1
                    break
                end
            end

        end
        if findOK==1
            break
        end
    end
    
    if findOK==0
        for j=1:6
            if car_pos_y==(j-1)*L
                for i=1:5
                    if (car_pos_x>(i-1)*L) && (car_pos_x<i*L)
                        angle=car.Yaw;
                        findOK=1;
                        if mod(round(angle/90),4)==0
                            from_point_x=i;
                            from_point_y=j;
                            to_point_x=i+1;
                            to_point_y=j;
                        elseif mod(round(angle/90),4)==2
                            from_point_x=i+1;
                            from_point_y=j;
                            to_point_x=i;
                            to_point_y=j;
                        end
                    end
                    if findOK==1
                        break
                    end
                end
            end
            if findOK==1
                break
            end
        end
    end
    if findOK==0
        from_point=NaN;
        to_point=NaN;
    else
        from_point=nodes_indexpair2index(from_point_x,from_point_y);
        to_point=nodes_indexpair2index(to_point_x,to_point_y);
    end

end

function vector=r(car)
    if mod(round(car.Yaw/90),4)==0
        vector=[1;0;0];
    elseif mod(round(car.Yaw/90),4)==2
        vector=[-1;0;0];
    elseif mod(round(car.Yaw/90),4)==1
        vector=[0;1;0];
    elseif mod(round(car.Yaw/90),4)==3
        vector=[0;-1;0];
    end
end

% find the future possible roads a car may take and give them a weight
function [from_point_list,to_point_list]=spread_influence(from_point,to_point,nodes_index2indexpair,nodes_indexpair2index)
from_point_index_pair=cell2mat(nodes_index2indexpair(from_point));
from_x=from_point_index_pair(1);
from_y=from_point_index_pair(2);
to_point_index_pair=cell2mat(nodes_index2indexpair(to_point));
to_x=to_point_index_pair(1);
to_y=to_point_index_pair(2);

from_point_list=[];
to_point_list=[];

if from_x==to_x+1
    if to_x>1
        from_point_list=[from_point_list to_point];
        to_point_list=[to_point_list nodes_indexpair2index(to_x-1,to_y)];
    end
    if to_y>1
        from_point_list=[from_point_list to_point];
        to_point_list=[to_point_list nodes_indexpair2index(to_x,to_y-1)];
    end
    if to_y<6
        from_point_list=[from_point_list to_point];
        to_point_list=[to_point_list nodes_indexpair2index(to_x,to_y+1)];
    end
elseif from_x==to_x-1
    if to_x<6
        from_point_list=[from_point_list to_point];
        to_point_list=[to_point_list nodes_indexpair2index(to_x+1,to_y)];
    end
    if to_y>1
        from_point_list=[from_point_list to_point];
        to_point_list=[to_point_list nodes_indexpair2index(to_x,to_y-1)];
    end
    if to_y<6
        from_point_list=[from_point_list to_point];
        to_point_list=[to_point_list nodes_indexpair2index(to_x,to_y+1)];
    end
elseif from_y==to_y+1
    if to_y>1
        from_point_list=[from_point_list to_point];
        to_point_list=[to_point_list nodes_indexpair2index(to_x,to_y-1)];
    end
    if to_x>1
        from_point_list=[from_point_list to_point];
        to_point_list=[to_point_list nodes_indexpair2index(to_x-1,to_y)];
    end
    if to_x<6
        from_point_list=[from_point_list to_point];
        to_point_list=[to_point_list nodes_indexpair2index(to_x+1,to_y)];
    end
elseif from_y==to_y-1
    if to_y<6
        from_point_list=[from_point_list to_point];
        to_point_list=[to_point_list nodes_indexpair2index(to_x,to_y+1)];
    end
    if to_x>1
        from_point_list=[from_point_list to_point];
        to_point_list=[to_point_list nodes_indexpair2index(to_x-1,to_y)];
    end
    if to_x<6
        from_point_list=[from_point_list to_point];
        to_point_list=[to_point_list nodes_indexpair2index(to_x+1,to_y)];
    end
end
end  

% calculate the number of cars around an access point for calculating the lossing rate
function num_car= count_car(position,car,rayon,T0,k,fin)
num_car=0;
for i=1:width(car)
    car_pos=car(i).Position;
    if norm(car_pos'-[position;0])<rayon && k>=T0(i) && fin(i)==0
        num_car=num_car+1;
    end
end
end