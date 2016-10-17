function hh = custom_trisurf(tri,varargin)
%TRISURF Triangular surface plot
%   TRISURF(TRI,X,Y,Z,C) displays the triangles defined in the M-by-3
%   face matrix TRI as a surface.  A row of TRI contains indexes into
%   the X,Y, and Z vertex vectors to define a single triangular face.
%   The color is defined by the vector C.
%
%   TRISURF(TRI,X,Y,Z) uses C = Z, so color is proportional to surface
%   height.
%
%   TRISURF(TR) displays the triangles in the triangulation TR. It uses
%   C = TR.X(:,3) to color the surface proportional to height.
%
%   H = TRISURF(...) returns a patch handle.
%
%   TRISURF(...,'param','value','param','value'...) allows additional
%   patch param/value pairs to be used when creating the patch object.
%
%   Example:
%
%   [x,y] = meshgrid(1:15,1:15);
%   tri = delaunay(x,y);
%   z = peaks(15);
%   trisurf(tri,x,y,z)
%
%   % Alternatively
%   tr = triangulation(tri, x(:), y(:), z(:));
%   trisurf(tr)
%
%   See also PATCH, TRIMESH, DELAUNAY, triangulation, delaunayTriangulation.

%   Copyright 1984-2015 The MathWorks, Inc.

ax = axescheck(varargin{:});
ax = newplot(ax);

x = varargin{1};
y = varargin{2};
z = varargin{3};
trids = tri;
c = z;
old_h=varargin{4};

h = set(old_h,'faces',trids,'vertices',[x(:) y(:) z(:)],'facevertexcdata',c(:),...
    'facecolor',get(ax,'DefaultSurfaceFaceColor'), ...
    'edgecolor',get(ax,'DefaultSurfaceEdgeColor'),'parent',ax);
end