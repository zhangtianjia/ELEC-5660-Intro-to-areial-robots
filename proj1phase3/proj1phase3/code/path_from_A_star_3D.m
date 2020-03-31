function Optimal_path = path_from_A_star_3D(map)

Optimal_path = [];
size_map = size(map,1);

MAX_X=10;
MAX_Y=10;
MAX_Z=10;

%Define the 3D grid map array.
%Obstacle=-1, Target = 0, Start=1
MAP=2*(ones(MAX_X,MAX_Y,MAX_Z));

%Initialize MAP with location of the target
xTarget=floor(map(size_map, 1));
yTarget=floor(map(size_map, 2));
zTarget=floor(map(size_map, 3));
MAP(xTarget,yTarget,zTarget)=0;

%Initialize MAP with location of the obstacle
for i = 2: size_map-1
    xval=floor(map(i, 1));
    yval=floor(map(i, 2));
    zval=floor(map(i, 3));
    MAP(xval,yval,zval)=-1;
end

%Initialize MAP with location of the start point
xStart=floor(map(1, 1));
yStart=floor(map(1, 2));
zStart=floor(map(1, 3));
MAP(xStart,yStart, zStart)=1;
%start your code
OPEN_list = [];
CLOSED_list = [];
k = 1;%Dummy counter
for i=1:MAX_X
    for j=1:MAX_Y
        for n=1:MAX_Z
            if(MAP(i,j,n) == -1)
                CLOSED_list(k,1) = i;
                CLOSED_list(k,2)=j;
                CLOSED_list(k,3)=n;
                k=k+1;
            end
        end
    end
end
CLOSED_COUNT = size(CLOSED_list,1);
xNode = xStart;
yNode = yStart;
zNode = zStart;
OPEN_COUNT = 1;
path_cost = 0;
goal_distance = sqrt((xTarget-xStart)^2 + (yTarget-yStart)^2 + (zTarget-zStart)^2);
OPEN_list(OPEN_COUNT,:) = [1 xNode yNode zNode xNode yNode zNode path_cost goal_distance, goal_distance];
CLOSED_COUNT = CLOSED_COUNT + 1;
CLOSED_list(CLOSED_COUNT,1) = xNode;
CLOSED_list(CLOSED_COUNT,2) = yNode;
CLOSED_list(CLOSED_COUNT,3) = zNode;
NoPath = 1;
while ((xNode ~= xTarget || yNode ~= yTarget || zNode ~= zTarget) &&NoPath == 1)
    expand = [];
    expand_num = 1;
    close_num = size(CLOSED_list,1);
    neighbor = [-1 0 1 -1 0 1 -1 0 1 -1 0 1 -1  1 -1 0 1 -1 0 1 -1 0 1 -1 0 1;
                1 1 1 0 0 0 -1 -1 -1  1 1 1 0  0 -1 -1 -1  1 1 1 0 0 0 -1 -1 -1;
                -1 -1 -1 -1 -1 -1 -1 -1 -1 0 0  0 0 0 0 0 0 1 1 1 1 1 1 1 1 1;];
    neighbor_count = size(neighbor, 2);
    for i = 1:neighbor_count
        x_new = xNode + neighbor(1,i);
        y_new = yNode + neighbor(2,i);
        z_new = zNode + neighbor(3,i);
        if ((x_new >0 && x_new <=MAX_X) && (y_new >0 && y_new <= MAX_Y)&& (z_new >0 && z_new <= MAX_Z))
            flag = 1;
            for j = 1:close_num
                if (x_new == CLOSED_list(j,1) && y_new ==CLOSED_list(j,2) && z_new ==CLOSED_list(j,3))
                    flag =0;
                end
            end
            if flag == 1
                expand(expand_num, 1) = x_new;
                expand(expand_num, 2) = y_new;
                expand(expand_num, 3) = z_new;
                expand(expand_num, 4) = path_cost + sqrt((xNode - x_new)^2 + (yNode - y_new)^2 + (zNode - z_new)^2);
                expand(expand_num, 5) = sqrt((xTarget - x_new)^2 + (yTarget - y_new)^2 + (zTarget - z_new)^2);
                expand(expand_num, 6) = expand(expand_num, 4) + expand(expand_num, 5);
                expand_num = expand_num + 1;
            end
        end
    end
    expand_size = size(expand,1);
    for i = 1:expand_size
        flag = 0;
        for j = 1:OPEN_COUNT
            if (expand(i,1) == OPEN_list(j,2) && expand(i,2) == OPEN_list(j,3) && expand(i,3) == OPEN_list(j,4))
                OPEN_list(j,10) = min(OPEN_list(j,10),expand(i,6));
                if OPEN_list(j,10) == expand(i,6)
                    OPEN_list(j,5) = xNode;
                    OPEN_list(j,6) = yNode;
                    OPEN_list(j,7) = zNode;
                    OPEN_list(j,8) = expand(i,4);
                    OPEN_list(j,9) = expand(i,5);
                end
                flag = 1;
            end
            
        end
        if flag ==0
            OPEN_COUNT = OPEN_COUNT + 1;
            OPEN_list(OPEN_COUNT, :) = [1,expand(i,1), expand(i,2),expand(i,3),xNode, yNode,zNode,expand(i,4), expand(i,5),expand(i,6)];
        end
    end
    temp_array=[];
    k=1;
    flag=0;
    goal_index=0;
    for j=1:OPEN_COUNT
        if (OPEN_list(j,1)==1)
            temp_array(k,:)=[OPEN_list(j,:) j];
            if (OPEN_list(j,2)==xTarget && OPEN_list(j,3)==yTarget && OPEN_list(j,4)==zTarget)
                flag=1;
                goal_index=j;
            end
            k=k+1;
        end
    end
    if flag == 1
        i_min=goal_index;
    end
    if size(temp_array ~= 0)
        [min_fn,temp_min]=min(temp_array(:,10));
        i_min=temp_array(temp_min,11);
    else
        i_min=-1;
    end
    if (i_min ~= -1)
        xNode = OPEN_list(i_min,2);
        yNode = OPEN_list(i_min,3);
        zNode = OPEN_list(i_min,4);
        path_cost = OPEN_list(i_min,8);
        CLOSED_COUNT = CLOSED_COUNT+1;
        CLOSED_list(CLOSED_COUNT,1) = xNode;
        CLOSED_list(CLOSED_COUNT,2) = yNode;
        CLOSED_list(CLOSED_COUNT,3) = zNode;
        OPEN_list(i_min,1) = 0;
    else
        NoPath = 0;
    end
end

i=size(CLOSED_list,1);
Optimal_path=[];
xval=CLOSED_list(i,1);
yval=CLOSED_list(i,2);
zval=CLOSED_list(i,3);
i=1;
Optimal_path(i,1)=xval;
Optimal_path(i,2)=yval;
Optimal_path(i,3)=zval;
i=i+1;
if ( (xval == xTarget) && (yval == yTarget) && (zval == zTarget))
    %Traverse OPEN and determine the parent nodes
    j = 1;
    while (OPEN_list(j, 2) ~= xval || OPEN_list(j,3) ~= yval || OPEN_list(j,4) ~= zval)
        j=j+1;
    end
    parent_x=OPEN_list(j,5);
    parent_y=OPEN_list(j,6);
    parent_z=OPEN_list(j,7);
    
    while( parent_x ~= xStart || parent_y ~= yStart || parent_z ~= zStart)
        Optimal_path(i,1) = parent_x;
        Optimal_path(i,2) = parent_y;
        Optimal_path(i,3) = parent_z;
        %Get the grandparents:-)
        k = 1;
        while (OPEN_list(k,2) ~= parent_x || OPEN_list(k,3) ~= parent_y ||OPEN_list(k,4) ~= parent_z)
            k = k+1;
        end
        parent_x=OPEN_list(k,5);%node_index returns the index of the node
        parent_y=OPEN_list(k,6);
        parent_z=OPEN_list(k,7);
        i=i+1;
    end
end
Optimal_path = [Optimal_path;xStart,yStart,zStart];
Optimal_path = Optimal_path(end:-1:1,:);
end




