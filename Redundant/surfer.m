stiffness=force./-(z-meanz);
triInit=delaunay(x,y);
figure;trisurf(triInit,x,y,abs(stiffness))
title('Stiffness Map');
shading interp
figure;trisurf(triInit,x,y,abs(force))
shading interp
title('Force Map');