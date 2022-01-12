classdef MyGraph
    
    properties
        Height
        Width
        MapGrid
        ObstacleProportion
        
        Vertices
        VertexIDMat
        VertexNum
               
        DistMat        
    end
    
    methods
        function obj = MyGraph(height,width,prop)
            obj.Height = height;
            obj.Width = width;
            mapGrid = zeros(height,width);
            obj.ObstacleProportion = prop;            
            totalNum = height*width;
            removedNum = round(height*width*prop);
            obj.VertexNum = totalNum-removedNum;
            perm = randperm(totalNum);
            for i = 1:removedNum
                temp = perm(1,i);
                c = mod(temp-1,width)+1;
                r = fix((temp-1)/width)+1;
                mapGrid(r,c) = 1;                                
            end
            obj.MapGrid = mapGrid;
            
            %vertices
            temp = 1;
            vertexMat = zeros(obj.VertexNum,2);
            mat = zeros(height,width);
            for i=1:height
                for j=1:width
                    if mapGrid(i,j)==0
                        vertexMat(temp,:) = [i,j];
                        mat(i,j) = temp;
                        temp = temp + 1;
                    end                    
                end
            end
            
            obj.Vertices = vertexMat;
            obj.VertexIDMat = mat;
            distMat = zeros(obj.VertexNum);
            
            for i=1:obj.VertexNum
                for j=i:obj.VertexNum
                    if i==j
                        distMat(i,j) = 0;
                    else
                        disp(i);
                        disp(j);
                        start = obj.Vertices(i,:);
                        goal = obj.Vertices(j,:);
                        if obj.ObstacleProportion == 0
                            distMat(i,j) = abs(start(1,1)-goal(1,1))+abs(start(1,2)-goal(1,2));
                        else
                            path = AStarST(mapGrid,start,goal,[]);
                            distMat(i,j) = size(path,1)-1;
                        end
                        
                        
                    end                                       
                end
            end
            
            for i=1:obj.VertexNum
                for j=1:i-1
                    distMat(i,j) = distMat(j,i);
                end
            end
            
            obj.DistMat = distMat;
            
        end        
    end
end

