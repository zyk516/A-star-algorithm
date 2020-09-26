function path = A_star_search(map,MAX_X,MAX_Y)
%addpath('E:\运动规划\第二节\hw_2\matlab版本作业\code\A_star')
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1);
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1,normal=2
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xval;
    yNode=yval;
    OPEN_COUNT=1;
    goal_distance=distance(xNode,yNode,xTarget,yTarget);
    path_cost=0;
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
    OPEN(OPEN_COUNT,1)=0;
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;

%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %Initialize all nodes.For every node,its g-value is Inf,and its h-value
    %is calculated by Euclidean distance between the node and the target
    %node.For begin node,its g-value is initialized 0.
    init_state=ones(MAX_X,MAX_Y,4);
    for i=1:MAX_X
        for j=1:MAX_Y
            init_state(i,j,1)=i;
            init_state(i,j,2)=j;
            init_state(i,j,3)=Inf;%g
            init_state(i,j,4)=distance(i,j,xTarget,yTarget);%h
        end
    end
    %To expand all neigh nodes of the begin node.First you need to find
    %dicrections that you can expand.Second,expand all nodes according to
    %right dicrections that judged by last step.
    init_state(xStart,yStart,3)=0;
    min_f=OPEN(1,8);
    min_f_index=1;
    x_now_node=OPEN(min_f_index,2);
    y_now_node=OPEN(min_f_index,3);
    g_now_node=OPEN(min_f_index,7);
    %The dicrection that you can expand.If you expand node by one
    %dicrection,you will run into an obstacle or go over the line or the node is already expanded ,this
    %dicrection will be seted [0,0].
    dicrection=[-1,-1;-1,0;-1,1;0,-1;0,0;0,1;1,-1;1,0;1,1];
    for i=1:1:9
        x_neigh=x_now_node+dicrection(i,1);
        if x_neigh<=0||x_neigh>MAX_X
            dicrection(i,1)=0;dicrection(i,2)=0;
            continue
        end
        y_neigh=y_now_node+dicrection(i,2);
        if x_neigh==x_now_node&&y_neigh==y_now_node
            continue
        end
        if y_neigh<=0||y_neigh>MAX_Y
            dicrection(i,1)=0;dicrection(i,2)=0;
            continue
        end
        already_expand=1;
        for k=1:CLOSED_COUNT
            if CLOSED(k,1)==x_neigh&&CLOSED(k,2)==y_neigh
                already_expand=2;
                break;
            end
        end
        if already_expand==2
            dicrection(i,1)=0;dicrection(i,2)=0;
            continue;
        end
        if MAP(x_neigh,y_neigh)==-1
            if x_neigh-x_now_node==1&&y_neigh-y_now_node==1 dicrection(9,1)=0;dicrection(9,2)=0; end
            if x_neigh-x_now_node==1&&y_neigh-y_now_node==0 dicrection(7,1)=0;dicrection(7,2)=0; end
            if x_neigh-x_now_node==0&&y_neigh-y_now_node==1 dicrection(3,1)=0;dicrection(3,2)=0; end
            if x_neigh-x_now_node==0&&y_neigh-y_now_node==0 dicrection(1,1)=0;dicrection(1,2)=0; end
        end
    end
    %loop all dicrections that is not [0,0].
    for i=1:1:9
        if dicrection(i,1)==0&&dicrection(i,2)==0
            continue
        end
        x_neigh=x_now_node+dicrection(i,1);
        y_neigh=y_now_node+dicrection(i,2);
        h1=init_state(x_neigh,y_neigh,4);
        %g-value==Inf,update its h,g and f value
        if init_state(x_neigh,y_neigh,3)==Inf
            OPEN_COUNT=OPEN_COUNT+1;
            g1=g_now_node+distance(x_neigh,y_neigh,x_now_node,y_now_node);
            f1=g1+h1;
            init_state(x_neigh,y_neigh,3)=g1;
            OPEN(OPEN_COUNT,:)=insert_open(x_neigh,y_neigh,x_now_node,y_now_node,h1,g1,f1);
        end
        %If 
        if init_state(x_neigh,y_neigh,3)>g_now_node+distance(x_neigh,y_neigh,x_now_node,y_now_node)
            init_state(x_neigh,y_neigh,3)=g_now_node+distance(x_neigh,y_neigh,x_now_node,y_now_node);
        end
    end
    while(1)
        if size(OPEN,1)==0
            break;
        end
        %Find nodes with the least f-value and put it to the closed list
        for i=1:OPEN_COUNT
            if OPEN(i,1)~=0
                min_f=OPEN(i,8);
                min_f_index=i;
                break;
            end
        end
        for i=min_f_index+1:OPEN_COUNT
            if OPEN(i,1)==0
                continue;
            end
            if min_f>OPEN(i,8)
                min_f=OPEN(i,8);
                min_f_index=i;
            end
        end
        OPEN(min_f_index,1)=0;
        CLOSED_COUNT=CLOSED_COUNT+1;
        CLOSED(CLOSED_COUNT,1)=OPEN(min_f_index,2);
        CLOSED(CLOSED_COUNT,2)=OPEN(min_f_index,3);
        %If you find the target node,end loop
        if OPEN(min_f_index,2)==xTarget&&OPEN(min_f_index,3)==yTarget
            break
        end
        %the following node is same as lines 104-157,and their function is
        %same
        x_now_node=OPEN(min_f_index,2);
        y_now_node=OPEN(min_f_index,3);
        g_now_node=OPEN(min_f_index,7);
        dicrection=[-1,-1;-1,0;-1,1;0,-1;0,0;0,1;1,-1;1,0;1,1];
        %             1     2    3   4    5   6   7    8   9
        for i=1:1:9
            x_neigh=x_now_node+dicrection(i,1);
            if x_neigh<=0||x_neigh>MAX_X
                dicrection(i,1)=0;dicrection(i,2)=0;
                continue
            end
            y_neigh=y_now_node+dicrection(i,2);
            if x_neigh==x_now_node&&y_neigh==y_now_node
                continue
            end
            if y_neigh<=0||y_neigh>MAX_Y
                dicrection(i,1)=0;dicrection(i,2)=0;
                continue
            end
            already_expand=1;
            for k=1:CLOSED_COUNT
                if CLOSED(k,1)==x_neigh&&CLOSED(k,2)==y_neigh
                    already_expand=2;
                    break;
                end
            end
            if already_expand==2
                dicrection(i,1)=0;dicrection(i,2)=0;
                continue;
            end
            if MAP(x_neigh,y_neigh)==-1
                if x_neigh-x_now_node==1&&y_neigh-y_now_node==1 dicrection(9,1)=0;dicrection(9,2)=0; end
                if x_neigh-x_now_node==1&&y_neigh-y_now_node==0 dicrection(7,1)=0;dicrection(7,2)=0; end
                if x_neigh-x_now_node==0&&y_neigh-y_now_node==1 dicrection(3,1)=0;dicrection(3,2)=0; end
                if x_neigh-x_now_node==0&&y_neigh-y_now_node==0 dicrection(1,1)=0;dicrection(1,2)=0; end
            end
        end
        for i=1:1:9
            if dicrection(i,1)==0&&dicrection(i,2)==0
                continue
            end
            x_neigh=x_now_node+dicrection(i,1);         
            y_neigh=y_now_node+dicrection(i,2);
            h1=init_state(x_neigh,y_neigh,4);
            if init_state(x_neigh,y_neigh,3)==Inf
                OPEN_COUNT=OPEN_COUNT+1;
                g1=g_now_node+distance(x_neigh,y_neigh,x_now_node,y_now_node);
                f1=g1+h1;
                OPEN(OPEN_COUNT,:)=insert_open(x_neigh,y_neigh,x_now_node,y_now_node,h1,g1,f1);
            end
            if init_state(x_neigh,y_neigh,3)>g_now_node+h1
                init_state(x_neigh,y_neigh,3)=g_now_node+h1;
                for k=1:OPEN_COUNT
                    if OPEN(k,2)==x_neigh&&OPEN(k,3)==y_neigh&&OPEN(k,1)==1
                        OPEN(k,7)=g_now_node+h1;
                        OPEN(k,8)=g_now_node+h1+OPEN(k,6);
                    end
                end
            end
        end              
     %
     %finish the while loop
     % 
    end %End of While Loop 
    %generate optimal path
    path=[];
    %将终点及其父节点加到路径中
    path_node=0;
    path_node=path_node+1;
    path(path_node,1)=xTarget-X_offset-0.5;
    path(path_node,2)=yTarget-Y_offset-0.5;
    parent_x=OPEN(OPEN_COUNT,4);
    parent_y=OPEN(OPEN_COUNT,5);
    path_node=path_node+1;
    path(path_node,1)=parent_x-X_offset-0.5;
    path(path_node,2)=parent_y-Y_offset-0.5; 
    for i=OPEN_COUNT-1:-1:1
        if OPEN(i,2)==parent_x&&OPEN(i,3)==parent_y
           parent_x=OPEN(i,4);
           parent_y=OPEN(i,5);
           path_node=path_node+1;
           path(path_node,1)=parent_x-X_offset-0.5;
           path(path_node,2)=parent_y-Y_offset-0.5;
           if parent_x==xStart&&parent_y==yStart
               break               
           end
           parent_x=OPEN(i,4);
           parent_y=OPEN(i,5);
        end
    end   
end 
