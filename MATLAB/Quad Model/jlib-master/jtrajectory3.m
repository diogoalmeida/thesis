function [h, M] = jtrajectory3(x,y,z,pitch,roll,yaw,scale_factor,step,body_model,theView,linespec,face_linespec,edge_linespec)
% function [M] = jtrajectory3(x,y,z,pitch,roll,yaw,scale_factor,step,body_model,theView,linespec,face_linespec,edge_linespec)
%   
%
%   x,y,z               center trajectory (vector)    [m]
%   
%   pitch,roll,yaw      euler's angles                [rad]
%   
%   scale_factor        normalization factor          [scalar]
%                              (related to body aircraft dimension)
%   
%   step                attitude sampling factor      [scalar]
%                              (the points number between two body models)
%   
%   body_model          select the body model         [string]
%
%                       none    No body model
%
%                       gripen  JAS 39 Gripen            heli        Helicopter
%                       mig     Mig			             ah64        Apache helicopter
%                       tomcat  Tomcat(Default)          a10
%                       jet     Generic jet		         cessna      Cessna
%                       747     Boeing 747		         biplane     Generic biplane
%                       md90    MD90 jet		         shuttle     Space shuttle
%                       dc10    DC-10 jet                mikrikopter Quadrotor
%
% OPTIONAL INPUT:
%
%
%    theView            sets the camera view. Use Matlab's "view" as
%                       argument to reuse the current view.
%    linespec           LineSpec for the trajectory line appearance.
%    face_linespec      LineSpec for the model faces appearance.
%    edge_linespec      LineSpec for the model edge appearance.
%
% OPTIONAL OUTPUTS:
%
%    M                  If present, animate the path and produce a movie
%
% Refernce System:
%                       X body- The axial force along the X body  axis is
%                       positive along forward; the momentum around X body
%                       is positive roll clockwise as viewed from behind;
%                       Y body- The side force along the Y body axis is
%                       positive along the right wing; the moment around Y
%                       body is positive in pitch up;
%                       Z body- The normal force along the Z body axis is
%                       positive down; the moment around Z body is positive
%                       roll clockwise as viewed from above.

DEFAULT_BODY_MODEL = 'gripen';

if nargin<9
    disp('Error: Invalid Number Inputs!'); disp(nargin)
    M=0;
    return;
end
if (length(x)~=length(y))||(length(x)~=length(z))||(length(y)~=length(z))
    disp('Error: Incorrect Dimension of the center trajectory Vectors. Please Check the size');
    M=0;
    return;
end
if ((length(pitch)~=length(roll))||(length(pitch)~=length(yaw))||(length(roll)~=length(yaw)))
    disp('Error: Incorrect Dimension of the euler''s angle Vectors. Please Check the size');
    M=0;
    return;
end
if length(pitch)~=length(x)
    disp('Error: Size mismatch between euler''s angle vectors and center trajectory vectors');
    M=0;
    return
end
if step>=length(x)
    disp('Error: Attitude sampling factor out of range. Reduce step');
    M=0;
    return
end
if step<1
    step=1;
end
if nargin<10
    theView=[82.50 2];
end
if nargin<11
    linespec='b';
end
if nargin<12
    face_linespec = 'r';
end
if nargin<13
    edge_linespec = 'none';
end

%are we animating/making a movie?
mov=nargout>1;

if strcmp(body_model,'none')
    V=[];
else
    mydir = fileparts(mfilename('fullpath'));
    body_matfile = sprintf('%s/traj_models/%s.mat', mydir, body_model);
    try
        %load the vertices and the faces from the model
        load(body_matfile, 'F', 'V');
    catch %#ok<CTCH>
        warning('Model: %s not found', body_model);
        load(sprintf('%s/traj_models/%s.mat', mydir, DEFAULT_BODY_MODEL), 'F', 'V');
        body_model = DEFAULT_BODY_MODEL;
    end
end

%swap axes of models so they display uniformy, as appropriate
if strcmp(body_model,'shuttle')
    V=[-V(:,2) V(:,1) V(:,3)];
elseif strcmp(body_model,'helicopter')
    V=[-V(:,2) V(:,1) V(:,3)];
elseif strcmp(body_model,'747')
    V=[V(:,2) V(:,1) V(:,3)];
elseif strcmp(body_model,'biplane')
    V=[-V(:,2) V(:,1) V(:,3)];
elseif strcmp(body_model,'md90')
    V=[-V(:,1) V(:,2) V(:,3)];
elseif strcmp(body_model,'dc10')
    V=[V(:,2) V(:,1) V(:,3)];
elseif strcmp(body_model,'ah64')
    V=[V(:,2) V(:,1) V(:,3)];
elseif strcmp(body_model,'mig')
    V=[V(:,2) V(:,1) V(:,3)];
elseif strcmp(body_model,'tomcat')
    V=[-V(:,2) V(:,1) V(:,3)];
elseif strcmp(body_model,'jet')
    V=[-V(:,2) V(:,1) V(:,3)];
elseif strcmp(body_model,'cessna')
    V=[-V(:,2) V(:,1) V(:,3)];
elseif strcmp(body_model,'A-10')
    V=[V(:,3) V(:,1) V(:,2)];
elseif strcmp(body_model,'gripen')
    V=[-V(:,1) -V(:,2) V(:,3)];
elseif strcmp(body_model,'mikrokopter')
    V=[V(:,1) V(:,2) -V(:,3)];
elseif strcmp(body_model,'none')
    %pass
else
    assert(false, sprintf('Model: %s not found', body_model));
end

if ~isempty(V)
    % scale the body model so it will be displayed correctly by patch
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
    correction=max(abs(V(:,1)));
    V=V./(scale_factor*correction);
end
  
y=y;
z=z;
pitch=pitch;
roll=-roll;
yaw=-yaw;
frame = 0;
ii=length(x);
resto=mod(ii,step);

% plot each body model
for i=1:step:(ii-resto)
    if mov || (i == 1)
      %clf;
      h = plot3(x,y,z,linespec,'LineWidth',1);
      grid on;
      hold on;
      light;
    end

    if ~isempty(V)
        % rotation matrix according to yaw
        theta=pitch(i);
        phi=roll(i);
        psi=yaw(i);
        Tbe=[cos(psi)*cos(theta), -sin(psi)*cos(theta), sin(theta);
                cos(psi)*sin(theta)*sin(phi)+sin(psi)*cos(phi) ...
                -sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) ...
                -cos(theta)*sin(phi);
                -cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi) ...
                sin(psi)*sin(theta)*cos(phi)+cos(psi)*sin(phi) ...
                cos(theta)*cos(phi)];

        %rotate and create the patch
        Vnew=V*Tbe;
        rif=[x(i) y(i) z(i)];
        X0=repmat(rif,size(Vnew,1),1);
        Vnew=Vnew+X0;
        p=patch('faces', F, 'vertices' ,Vnew);
        set(p, 'facec', face_linespec);          
        set(p, 'EdgeColor',edge_linespec); 
    end

    % set the view angle
    if mov || (i == 1)
        view(theView);
        axis equal;
    end
    % if creating a movie, repsect the angle and save the 
    % plot as a frame in the movie
    if mov
        if i == 1
            ax = axis;
        else
            axis(ax);
        end
        lighting phong
        frame = frame + 1;
        M(frame) = getframe;
    end

end

% plot the trajectory
%hold on;
%plot3(x,y,z);
lighting phong;
grid on;
view(theView);
daspect([1 1 1]) ;
xlabel('X');
ylabel('Y');
zlabel('Z');