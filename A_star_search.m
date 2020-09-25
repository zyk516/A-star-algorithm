function path = A_star_search(map,MAX_X,MAX_Y)
%待改进的地方：
%1.在寻找障碍物时没有顺便把超出边界的点挑出来
%2.也没有把已经扩展过的节点挑出来
addpath('E:\运动规划\第二节\hw_2\matlab版本作业\code\A_star')
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
    init_state=ones(MAX_X,MAX_Y,4);
    for i=1:MAX_X
        for j=1:MAX_Y
            init_state(i,j,1)=i;
            init_state(i,j,2)=j;
            init_state(i,j,3)=Inf;%g
            init_state(i,j,4)=distance(i,j,xTarget,yTarget);%h
        end
    end
    %遍历第一个节点的周围节点
    init_state(xStart,yStart,3)=0;
    min_f=OPEN(1,8);
    min_f_index=1;
    x_now_node=OPEN(min_f_index,2);
    y_now_node=OPEN(min_f_index,3);
    g_now_node=OPEN(min_f_index,7);
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
    for i=1:1:9
        if dicrection(i,1)==0&&dicrection(i,2)==0
            continue
        end
        x_neigh=x_now_node+dicrection(i,1);
        y_neigh=y_now_node+dicrection(i,2);
        h1=init_state(x_neigh,y_neigh,4);
        if init_state(x_neigh,y_neigh,3)==Inf
            OPEN_COUNT=OPEN_COUNT+1;
            %相邻节点计算距离
            g1=g_now_node+distance(x_neigh,y_neigh,x_now_node,y_now_node);
            f1=g1+h1;
            init_state(x_neigh,y_neigh,3)=g1;
            OPEN(OPEN_COUNT,:)=insert_open(x_neigh,y_neigh,x_now_node,y_now_node,h1,g1,f1);
        end
        if init_state(x_neigh,y_neigh,3)>g_now_node+distance(x_neigh,y_neigh,x_now_node,y_now_node)
            init_state(x_neigh,y_neigh,3)=g_now_node+distance(x_neigh,y_neigh,x_now_node,y_now_node);
            %不会存在已经被其他节点遍历过的节点，因为此时是对第一个节点的周围节点进行遍历
        end
    end
    ii=0;
    while(1) %you have to dicide the Conditions for while loop exit 
        ii=ii+1;
        if ii==1
        end
        if size(OPEN,1)==0
            break;
        end%?????????OPEN的大小不会为0
        %从OPEN里面寻找f最小的未被扩展的节点，并放到CLOSED中
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
        %遇到目标节点搜索结束
        if OPEN(min_f_index,2)==xTarget&&OPEN(min_f_index,3)==yTarget
            break
        end
        %遍历当前节点的周围节点
        x_now_node=OPEN(min_f_index,2);
        y_now_node=OPEN(min_f_index,3);
        g_now_node=OPEN(min_f_index,7);
        %寻找周围的障碍物，为下一步的搜索确定方向
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
            if x_neigh<=0||x_neigh>MAX_X
                continue
            end            
            y_neigh=y_now_node+dicrection(i,2);
            if y_neigh<=0||y_neigh>MAX_Y
                continue
            end
            %不在访问已经被扩展过的节点
             already_expand=1;
             for k=1:CLOSED_COUNT
                if CLOSED(k,1)==x_neigh&&CLOSED(k,2)==y_neigh
                    already_expand=2;
                    break;
                end
             end
             if already_expand==2
                continue;
             end
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
    
    %Once algorithm has run The optimal path is generated by starting of at the
    %last node(if it is the target node) and then identifying its parent node
    %until it reaches the start node.This is the optimal path
    
    %
    %How to get the optimal path after A_star search?
    %please finish it
    %
    path=[];
    %将终点及其父节点加到路径中
    path_node=0;
    path_node=path_node+1;
    path(path_node,1)=xTarget;
    path(path_node,2)=yTarget;
    parent_x=OPEN(OPEN_COUNT,4);
    parent_y=OPEN(OPEN_COUNT,5);
    path_node=path_node+1;
    path(path_node,1)=parent_x;
    path(path_node,2)=parent_y; 
    for i=OPEN_COUNT-1:-1:1
        if OPEN(i,2)==parent_x&&OPEN(i,3)==parent_y
           parent_x=OPEN(i,4);
           parent_y=OPEN(i,5);
           path_node=path_node+1;
           path(path_node,1)=parent_x;
           path(path_node,2)=parent_y;
           parent_x=OPEN(i,4);
           parent_y=OPEN(i,5);
        end
    end   
end
% for i=-1:1:1
%             x_neigh=x_now_node+i;
%             if x_neigh<=0||x_neigh>=MAX_X
%                 continue
%             end            
%             for j=-1:1:1
%                 y_neigh=y_now_node+j;
%                 if x_neigh==x_now_node&&y_neigh==y_now_node
%                     continue
%                 end
%                 if y_neigh<=0||y_neigh>=MAX_Y
%                     continue
%                 end
% %                 if MAP(x_neigh,y_neigh)==-1
% %                     continue
% %                 end
%                 %不能经过障碍物
%                 if (i==1&&j==1)||(i==1&&j==-1)||(i==-1&&j==1)||(i==-1&&j==-1)
%                     
%                 end
%                 %不在访问已经被扩展过的节点
%                 already_expand=1;
%                 for k=1:CLOSED_COUNT
%                     if CLOSED(k,1)==x_neigh&&CLOSED(k,2)==y_neigh
%                         already_expand=2;
%                         break;
%                     end
%                 end
%                 if already_expand==2
%                     continue;
%                 end
%                 h1=init_state(x_neigh,y_neigh,4);
%                 if init_state(x_neigh,y_neigh,3)==Inf
%                     OPEN_COUNT=OPEN_COUNT+1;
%                     g1=g_now_node+distance(x_neigh,y_neigh,x_now_node,y_now_node);
%                     f1=g1+h1;
%                     OPEN(OPEN_COUNT,:)=insert_open(x_neigh,y_neigh,x_now_node,y_now_node,h1,g1,f1);
%                 end
%                 if init_state(x_neigh,y_neigh,3)>g_now_node+h1
%                     init_state(x_neigh,y_neigh,3)=g_now_node+h1;
%                     for k=1:OPEN_COUNT
%                         if OPEN(k,2)==x_neigh&&OPEN(k,3)==y_neigh&&OPEN(k,1)==1
%                             OPEN(k,7)=g_now_node+h1;
%                             OPEN(k,8)=g_now_node+h1+OPEN(k,6);
%                         end
%                     end
%                 end                   
%             end
%         end              
