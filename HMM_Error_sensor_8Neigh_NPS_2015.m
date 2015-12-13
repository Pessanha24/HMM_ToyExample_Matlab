%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Nuno Pessanha Santos
%nuno.pessanha.santos@gmail.com
%nuno.pessanha.santos@marinha.pt
%https://marinha.academia.edu/NunoPessanhaSantos
%V01_A - Error Sensor Measurements
%8-Neighbourhood
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear all;
close all;

%%Sensor Noise
Threshold=0.85;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Save Video File
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vidObj = VideoWriter('Grid_Probab.avi');
vidObj.FrameRate=1;
open(vidObj);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Initial Scenario Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
%Grid Size
%Example: if 10 you need to put 11 for better visualitation
%All the calculations are made with this
%Grid_size = Size-1;
%&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
%Grid Size
Grid_size = 11;
% Grid_size = 21;

%Initial MeshGrid
%Plot
[X Y] = meshgrid(0:Grid_size);
figure(1), voronoi(X(:),Y(:)), axis square ;
set(gca,'YDir','Reverse');
title('Environment - Known');
axis([0 Grid_size 0 Grid_size]);
hold on
 
%&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
%Define Obstacles
%%Localization[column, Line]
%&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

%Obstacles Location
localization_obs = [3,1;
                    3,2;
                    3,4;
                    2,7;
                    2,9;
                    6,9;
                    4,9;
                    5,7;
                    9,4;
                    8,8;
                    8,7;
                    10,2];
% localization_obs = [3,1;
%                     3,2;
%                     3,4;
%                     2,7;
%                     2,9;
%                     6,9;
%                     4,9;
%                     5,7;
%                     9,4;
%                     8,8;
%                     8,7;
%                     10,2;
%                     18,2;
%                     15,8;
%                     12,9;
%                     17,5];                
                
%Number of obstacles
Size_localization_obs=size(localization_obs);

%££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££
%%Environment Matrix
%%1 possible state
%%0 obstacle
%Aux Matrix to obtain the state number
%££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££
Environment=ones(Grid_size-1);
  for k=1:Size_localization_obs(1)
      Environment(localization_obs(k,2),localization_obs(k,1))=0;   
  end
  %%State Number
  aux=1;
  for j=1:size(Environment,2)
    for k=1:size(Environment,1)
        if(Environment(k,j)==1)
            Environment(k,j)=aux;
            aux=aux+1;
        end         
      
    end
  end 
  
 %££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££
 %Plot red Circles - Obstacles
 %££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££
 for k=1:Size_localization_obs(1)
 plot(localization_obs(k,1),localization_obs(k,2),'r.','MarkerSize',(75/(Grid_size/10)));
 end

%££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Moving Directions
%Show thw moving directions in a plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££££

[X Y] = meshgrid(0:4);
figure(2), voronoi(X(:),Y(:)), axis square ;
title('Moving Directions / 4-Neighbourhood');
axis([0 4 0 4]);
hold on
 
%Plot Robot and Direction of sensing (sensor)
plot(2,2,'b.','MarkerSize',150);
plot(2,1,'g.','MarkerSize',150);
plot(1,2,'g.','MarkerSize',150);
plot(3,2,'g.','MarkerSize',150);
plot(2,3,'g.','MarkerSize',150);

plot(3,3,'g.','MarkerSize',150);
plot(3,1,'g.','MarkerSize',150);
plot(1,1,'g.','MarkerSize',150);
plot(1,3,'g.','MarkerSize',150);

%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
%Pause and close Figure 2
%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
%Pause until press a specific key
pause
%Close the Moving Directions Figure
close(figure(2))

%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Initialize Variables
%pi -> Initial probability
%A -> Transition Porbability
%B -> Output Model
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

%Total number of states
Total_number_States = ((Grid_size-1)^2)-Size_localization_obs(1);

%Initial probability vector
%Initially with equal probability for all the states
pi_v=ones(Total_number_States,1);
pi_v=pi_v/Total_number_States;

%The sum must be one
pi_v_sum=sum(pi_v);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Matrix A
%Available movements
%8-Neighbourhood
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for j=1:size(Environment,2)
    for k=1:size(Environment,1)
        if(Environment(k,j)~=0)
        if((k-1)>0 && Environment(k-1,j)~=0)
        teste(k-1,j)=1;
        A(Environment(k,j),1)=Environment(k-1,j);
        end
        if((k+1)<=(Grid_size-1)&& Environment(k+1,j)~=0)
        teste(k+1,j)=1;
        A(Environment(k,j),2)=Environment(k+1,j);
        end
        if((j-1)>0 && Environment(k,j-1)~=0)
        teste(k,j-1)=1;
        A(Environment(k,j),3)=Environment(k,j-1);
        end
        if((j+1)<=(Grid_size-1)&& Environment(k,j+1)~=0)
        teste(k,j+1)=1;
        A(Environment(k,j),4)=Environment(k,j+1);
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        if((k-1)>0 && (j-1)>0 && Environment(k-1,j-1)~=0)
        teste(k-1,j-1)=1;
        A(Environment(k,j),5)=Environment(k-1,j-1);
        end
        
        if((k-1)>0 && (j+1)<=(Grid_size-1) && Environment(k-1,j+1)~=0)
        teste(k-1,j+1)=1;
        A(Environment(k,j),6)=Environment(k-1,j+1);
        end
        
        if((k+1)<=(Grid_size-1) && (j-1)>0 && Environment(k+1,j-1)~=0)
        teste(k+1,j-1)=1;
        A(Environment(k,j),7)=Environment(k+1,j-1);
        end 
        
        if((k+1)<=(Grid_size-1)&& (j+1)<=(Grid_size-1) && Environment(k+1,j+1)~=0)
        teste(k+1,j+1)=1;
        A(Environment(k,j),8)=Environment(k+1,j+1);
        end
        
        
      end
    end
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Probability of each step
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
  aux=0;
   for j=1:size(A,1)
     for k=1:size(A,2)
        if(A(j,k)~=0)
            aux=aux+1;
        end
     end
     A(j,9)=1/aux;
     aux=0;
   end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Matrix B
%Sensor Output
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  for j=1:size(Environment,2)
    for k=1:size(Environment,1)
        if(Environment(k,j)~=0)
        if((k-1)>0 && Environment(k-1,j)~=0)
        teste(k-1,j)=1;
         B(Environment(k,j),1)=1;
        end
        if((k+1)<=(Grid_size-1)&& Environment(k+1,j)~=0)
        teste(k+1,j)=1;
         B(Environment(k,j),2)=1;
        end
        if((j-1)>0 && Environment(k,j-1)~=0)
        teste(k,j-1)=1;
         B(Environment(k,j),3)=1;
        end
        if((j+1)<=(Grid_size-1)&& Environment(k,j+1)~=0)
        teste(k,j+1)=1;
         B(Environment(k,j),4)=1;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        if((k-1)>0 && (j-1)>0 && Environment(k-1,j-1)~=0)
        teste(k-1,j-1)=1;
         B(Environment(k,j),5)=1;
        end
        
        if((j+1)<=(Grid_size-1) && (k-1)>0 && Environment(k-1,j+1)~=0)
        teste(k-1,j+1)=1;
         B(Environment(k,j),6)=1;
        end
        
        if((k+1)<=(Grid_size-1) && (j-1)>0 && Environment(k+1,j-1)~=0)
        teste(k+1,j-1)=1;
        B(Environment(k,j),7)=1;
        end 
        
        if((k+1)<=(Grid_size-1)&& (j+1)<=(Grid_size-1) && Environment(k+1,j+1)~=0)
        teste(k+1,j+1)=1;
        B(Environment(k,j),8)=1;
        end
               
        end
    end
  end
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %Convert Binary to Decimal
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 for j=1:size(B,1)
 B(j,9) = bi2de([B(j,1) B(j,2) B(j,3) B(j,4) B(j,5) B(j,6) B(j,7) B(j,8)],2,'left-msb');   
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Initial Robot Location
%Random - Based on initial Pi vector
cdf = cumsum(pi_v);
U = rand;
X = sum(cdf<=U)+1;
State(1,1)=X;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Return state coordinates in grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 for j=1:size(Environment,2)
    for k=1:size(Environment,1)
        if(Environment(k,j)==X)
            State(2,1) = k; %line
            State(2,2)=j;   %Column
        end
    end
 end
 
 %Plot Robot Position
 figure(1)
 plot(State(2,2), State(2,1),'g.','MarkerSize',(75/(Grid_size/10)));
 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%Given a coordinate return sensor value
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 k= State(2,1);
 j= State(2,2);
 if((k-1)>0 && Environment(k-1,j)~=0)
        State(3,1)=1;
        else
        State(3,1)=0;  
 end
        
        if((k+1)<=(Grid_size-1)&& Environment(k+1,j)~=0)
        State(3,2)=1;
        else
        State(3,2)=0;      
        end
        
        if((j-1)>0 && Environment(k,j-1)~=0)
        State(3,3)=1;
        else
        State(3,3)=0;    
        end    
        if((j+1)<=(Grid_size-1)&& Environment(k,j+1)~=0)
        State(3,4)=1;
        else
        State(3,4)=0;    
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        if((k-1)>0 && (j-1)>0 && Environment(k-1,j-1)~=0)
        State(3,5)=1;
        else
        State(3,5)=0;    
        end
        
        if((j+1)<=(Grid_size-1) && (k-1)>0 && Environment(k-1,j+1)~=0)
        State(3,6)=1;
        else
        State(3,6)=0;    
        end
        
        if((k+1)<=(Grid_size-1) && (j-1)>0 && Environment(k+1,j-1)~=0)
        State(3,7)=1;
        else
        State(3,7)=0;    
        end
        
        if((k+1)<=(Grid_size-1)&& (j+1)<=(Grid_size-1) && Environment(k+1,j+1)~=0)
        State(3,8)=1;
        else
        State(3,8)=0;    
        end
              
        
        
 %Convert Decimal
 State(4,1) = bi2de([State(3,1) State(3,2) State(3,3) State(3,4) State(3,5) State(3,6) State(3,7) State(3,8)],2,'left-msb');   
 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %Admiting  noise in the sensor read
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 for j=1:size(B,1)  
  if((B(j,9)) == State(4,1))
 %b_temp(j,1)=State(4,1);
 b_temp(j,1)=Threshold;
  else
 b_temp(j,1)=((1-Threshold)/((2^8)-1));
  end
 end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Compute first step - Baum Algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pi_1= b_temp.*pi_v;
pi_1_normalized=(b_temp.*pi_v)/(sum(b_temp.*pi_v));

%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
%Show Results in a Grid
%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
 for j=1:size(Environment,1)
      for i=1:(size(Environment,2))
        if(Environment(j,i)~=0)
        Show(j,i)=pi_1_normalized(Environment(j,i),1);
        end
      end
 end
 
%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
%Show HeatMap
%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

mincolor = min(Show(:));
maxcolor = max(Show(:));

figure(3)
heatmap(Show,[],[],'%0.2f','ColorMap', @cool, 'Colorbar',true, ...
    'MinColorValue', mincolor, 'MaxColorValue', maxcolor);
hold on
title('P(x|y)');
 plot(State(2,2), State(2,1),'g.','MarkerSize',(75/(Grid_size/10)))
  for k=1:Size_localization_obs(1)
 plot(localization_obs(k,1),localization_obs(k,2),'r.','MarkerSize',(75/(Grid_size/10)))
  end
  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Write each frame to the file
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
currFrame = getframe;
writeVideo(vidObj,currFrame);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Pause
%Show initial conditions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pause;

%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
%Iterative steps
%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%Robot Moves
%Dealing with the sensor information
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
for oo=1:1000
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %If the probability achieves one - stop the cycle
    %Perfect sensor
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(size(find(Show>0.95),1)>0)
        break
    end
%     
%Aux variable
pi_temppp=zeros(Total_number_States,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
%A'*alfa_t-1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 for j=1:size(A,1)
      for i=1:(size(A,2)-1)
         if(A(j,i)~=0)
         %pi_temppp(A(j,i),1)=pi_v(A(j,i),1)+(A(j,5)*pi_v(A(j,i),1));
         pi_temppp(A(j,i),1)=pi_temppp(A(j,i),1)+(pi_1_normalized(j,1)*A(j,5));
         end
      end
 end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Obtain next Robot move 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Next_Move=zeros(4,2);
aux_next_move=0;
for k=1:size(State,2)
   if(State(3,k)==1)
       Next_Move(k,1)=k;
         Next_Move(k,2)=1;
       aux_next_move=aux_next_move+1;
       
   end      
end
Next_Move(:,2)=Next_Move(:,2)/aux_next_move;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
%Rando move
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cdf_next_move = cumsum(Next_Move(:,2));
U_next_move = rand;
X_next_move = sum(cdf_next_move<=U_next_move)+1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Move
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
switch X_next_move
    case 1
        State(2,1)=State(2,1)-1;
        %disp('Um');
    case 2
        State(2,1)=State(2,1)+1;
        %disp('Dois');
    case 3
        State(2,2)=State(2,2)-1;
        %disp('Tres');
    case 4
        State(2,2)=State(2,2)+1; 
        %disp('Quatro');
    case 5
        State(2,1)=State(2,1)-1;
        State(2,2)=State(2,2)-1; 
        %disp('Quatro');
    case 6
        State(2,1)=State(2,1)-1;
        State(2,2)=State(2,2)+1;
%         disp('seis');
    case 7
        State(2,1)=State(2,1)+1;
        State(2,2)=State(2,2)-1;
%         disp('sete');
    case 8
        State(2,1)=State(2,1)+1;
        State(2,2)=State(2,2)+1;
%         disp('oito');
    otherwise
        disp('Error');
end

%Update State Variable
State(1,1)=Environment(State(2,1),State(2,2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%Given a coordinate return sensor value
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 k= State(2,1);
 j= State(2,2);
 if((k-1)>0 && Environment(k-1,j)~=0)
        State(3,1)=1;
        else
        State(3,1)=0;  
 end
        
        if((k+1)<=(Grid_size-1)&& Environment(k+1,j)~=0)
        State(3,2)=1;
        else
        State(3,2)=0;      
        end
        
        if((j-1)>0 && Environment(k,j-1)~=0)
        State(3,3)=1;
        else
        State(3,3)=0;    
        end    
        if((j+1)<=(Grid_size-1)&& Environment(k,j+1)~=0)
        State(3,4)=1;
        else
        State(3,4)=0;    
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        if((k-1)>0 && (j-1)>0 && Environment(k-1,j-1)~=0)
        State(3,5)=1;
        else
        State(3,5)=0;    
        end
        
        if((j+1)<=(Grid_size-1) && (k-1)>0 && Environment(k-1,j+1)~=0)
        State(3,6)=1;
        else
        State(3,6)=0;    
        end
        
        if((k+1)<=(Grid_size-1) && (j-1)>0 && Environment(k+1,j-1)~=0)
        State(3,7)=1;
        else
        State(3,7)=0;    
        end
        
        if((k+1)<=(Grid_size-1)&& (j+1)<=(Grid_size-1) && Environment(k+1,j+1)~=0)
        State(3,8)=1;
        else
        State(3,8)=0;    
        end
        
 %Convert Decimal
 State(4,1) = bi2de([State(3,1) State(3,2) State(3,3) State(3,4) State(3,5) State(3,6) State(3,7) State(3,8)],2,'left-msb');   
 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %Admiting  noise in the sensor read
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 for j=1:size(B,1)  
  if((B(j,9)) == State(4,1))
 %b_temp(j,1)=State(4,1);
 b_temp(j,1)=Threshold;
  else
 b_temp(j,1)=((1-Threshold)/((2^8)-1));
  end
 end

%Obtain alfa after one iteration

pi_1= b_temp.*pi_temppp;
pi_1_normalized=(pi_1)/(sum(pi_1));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Show Results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 for j=1:size(Environment,1)
      for i=1:(size(Environment,2))
            if(Environment(j,i)~=0)
                Show(j,i)=pi_1_normalized(Environment(j,i),1);    
            end
      end
 end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Mesh Grid
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 hold off 
 [X Y] = meshgrid(0:Grid_size);
 figure(1), voronoi(X(:),Y(:)), axis square ;
 set(gca,'YDir','Reverse');
 title('Environment - Known');
 axis([0 Grid_size 0 Grid_size]);
 hold on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot circles Obstacles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 for k=1:Size_localization_obs(1)
 plot(localization_obs(k,1),localization_obs(k,2),'r.','MarkerSize',(75/(Grid_size/10)));
 end
 plot(State(2,2), State(2,1),'g.','MarkerSize',(75/(Grid_size/10)));
 hold off
 
%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
%Show HeatMap
%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

mincolor = min(Show(:));
maxcolor = max(Show(:));

figure(3)
hold off
heatmap(Show,[],[],'%0.2f','ColorMap', @cool, 'Colorbar',true, ...
    'MinColorValue', mincolor, 'MaxColorValue', maxcolor);
hold on
title('P(x|y)');
 plot(State(2,2), State(2,1),'g.','MarkerSize',(75/(Grid_size/10)))
  for k=1:Size_localization_obs(1)
 plot(localization_obs(k,1),localization_obs(k,2),'r.','MarkerSize',(75/(Grid_size/10)))
  end

%pi_1_normalized
pause 
 
%Write each frame to the file.
currFrame = getframe;
writeVideo(vidObj,currFrame);

end

% Close the file.
close(vidObj);

