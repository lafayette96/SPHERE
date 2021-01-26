fis1 = sugfis;

fis1 = addInput(fis1,[-60 60],'Name','error');
fis1 = addInput(fis1,[-120 120],'Name','error_dervative');

fis1 = addMF(fis1,'error','trapmf',[-60 -60 -40 0],'Name','LOW');
fis1 = addMF(fis1,'error','trimf',[-40 0 40],'Name','MID');
fis1 = addMF(fis1,'error','trapmf',[0 40 60 60],'Name','HIGH');
fis1 = addMF(fis1,'error_dervative','trapmf', [-120 -120 -80 0],'Name','N');
fis1 = addMF(fis1,'error_dervative','trimf', [-80 0 80],'Name','Z');
fis1 = addMF(fis1,'error_dervative','trapmf', [0 80 120 120],'Name','P');

figure
subplot(1,2,1)
plotmf(fis1,'input',1)
title('Input 1')
subplot(1,2,2)
plotmf(fis1,'input',2)
title('Input 2')

fis1 = addOutput(fis1,[-60 60],'Name','control');

fis1 = addMF(fis1,'control','trapmf',[-60 -60 -40 0],'Name','LOW');
fis1 = addMF(fis1,'control','trimf',[-40 0 40],'Name','MID');
fis1 = addMF(fis1,'control','trapmf',[0 40 60 60],'Name','HIGH');
%fis1 = addMF(fis1,'U','constant',-1,'Name','NB');
%fis1 = addMF(fis1,'U','constant',-0.5,'Name','NM');
%fis1 = addMF(fis1,'U','constant',0,'Name','Z');
%fis1 = addMF(fis1,'U','constant',0.5,'Name','PM');
%fis1 = addMF(fis1,'U','constant',1,'Name','PB');

rules = [...
    "error==LOW => control=HIGH"; ...
    "error==MID => control=MID"; ...
    "error==HIGH => control=LOW"; ...
    "error==LOW => control=LOW"; ...
    "error==MID => control=MID"; ...
    "error==HIGH => control=HIGH"; ...
%    "E==N & delE==P => U=Z"; ...
%    "E==Z & delE==P => U=PM"; ...
%    "E==P & delE==P => U=PB" ...
    ];
fis1 = addRule(fis1,rules);

figure
gensurf(fis1)
title('Control surface of type-1 FIS')

fis2 = convertToType2(fis1);

scale = [0.2 0.9 0.2;0.3 0.9 0.3];
for i = 1:length(fis2.Inputs)
    for j = 1:length(fis2.Inputs(i).MembershipFunctions)
        fis2.Inputs(i).MembershipFunctions(j).LowerLag = 0;
        fis2.Inputs(i).MembershipFunctions(j).LowerScale = scale(i,j);
    end
end

figure
subplot(1,2,1)
plotmf(fis2,'input',1)
title('Input 1')
subplot(1,2,2)
plotmf(fis2,'input',2)
title('Input 2')

figure
gensurf(fis2)
title('Control surface of type-2 FIS')

%% PID

C = 0.5;
L = 0.5;
T = 0.5;
G = tf(C,[T 1],'Outputdelay',L);

pidController = pidtune(G,'pidf');
Ce = 1;

tauC = 0.2;

Cd = min(T,L/2)*Ce;
C0 = 1/(C*Ce*(tauC+L/2));
C1 = max(T,L/2)*C0;

model = 'comparepidcontrollers';
load_system(model)

%%Simulation
out1 = sim(model);
plotOutput(out1,['Nominal: C=' num2str(C) ', L='  num2str(L) ', T=' num2str(T)])
