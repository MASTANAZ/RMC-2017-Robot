function visualizeGrid(trueCells,trueRover,k,estiCells,estiRover)
  
  
  % True State
  [dims,xPatches,yPatches,rgbColors] = cells2patches(trueCells);
  
  
  subplot(2,1,1); 
  
  plot(dims(1,:), dims(2,:),'k-'); 
  axis('image'); 
  axis('manual');
  
  patch(yPatches',xPatches',permute(rgbColors,[3 1 2])); 
  
  title(['True State (Stage ' num2str(k) ')']);
  
  set(gca,'XTick',unique(dims(1,:)),'YTick',unique(dims(2,:)));
  
  vel = [cos(pi/180*trueRover.pos(3)) sin(pi/180*trueRover.pos(3)); 
        -sin(pi/180*trueRover.pos(3)) cos(pi/180*trueRover.pos(3))] * [0; 0.025];
        
  % Plot "bounding box" of rover
  hold on; plot(trueRover.pos(2),trueRover.pos(1),'rs','markersize',35, 'linewidth',2); hold off;
  
  hold on; plot(trueRover.pos(2),trueRover.pos(1),'bo','LineWidth',2); hold off; % circle
  hold on; plot(trueRover.pos(2)+[0 vel(2)],trueRover.pos(1)+[0 vel(1)],'b-','LineWidth',2); hold off; %line
  

  
  
  
  
  % Estimated State
  [dims,xPatches,yPatches,rgbColors] = cells2patches(estiCells);
  
  subplot(2,1,2); 
  plot(dims(1,:),dims(2,:),'k-'); 
  axis('image'); 
  axis('manual');
  
  patch(yPatches',xPatches',permute(rgbColors,[3 1 2])); 
  
  title(['Estimated State (Stage ' num2str(k) ')']);
  
  set(gca,'XTick',unique(dims(1,:)),'YTick',unique(dims(2,:)));
  
end



function [dims,xPatches,yPatches,rgbColors] = cells2patches(theCells)
  
  [M,N] = size(theCells);
  
  dims = [0 0 1 1 0; 0 M/N M/N 0 0];
  
  xPatches = nan(M*N,5); 
  yPatches = nan(M*N,5); 
  rgbColors = nan(M*N,3);
  
  for x = 1:M
    for y = 1:N
      pNdx = (x-1)*N + y;
      
      xPatches(pNdx,:) = (x-1)/N + [0 0 1/N 1/N 0];
      yPatches(pNdx,:) = (y-1)/N + [0 1/N 1/N 0 0];
      
      if theCells(x,y) == 1 % cell is within region 1 and not the unload station
        rgbColors(pNdx,:) = 0.5*[1 1 1];
      
      % cell is part of unload station (should only occur in region 1)
      elseif theCells(x,y) == 0 
        rgbColors(pNdx,:) = [0 1 0];
        
      % cell is within region 2 and absent of obstacle
      elseif floor(theCells(x,y)) == 2 
        
        % Easier obstacle
        if theCells(x,y) >= 2.5
          rgbColors(pNdx,:) = 0.5*[1 1 1] + (theCells(x,y)-2.5)*[1 1 1];

        % Harder obstacle
        else
          rgbColors(pNdx,:) = 0.5*[1 1 1] - (2.5-theCells(x,y))*[1 1 1]; 
          
        end
      
      % Impossible obstacle
      elseif isnan(theCells(x,y))
        rgbColors(pNdx,:) = [1 0 1]; 
        
        
      % cell is within region 3 and diggable
      elseif theCells(x,y) == 3 
        rgbColors(pNdx,:) = [1 1 1];
        
      end
      
    end
    
  end
  
end