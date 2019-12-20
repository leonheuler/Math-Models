%% Animation
fig6 = figure(6);
clf('reset');
human = animatedline('Marker','o','Color', 'black','LineWidth',1.25); hold on;    
exo =   animatedline('Marker','o','Color', 'blue','LineWidth',1.25);

tr = animatedline(); % end-effector trajectory
ecp = animatedline('Marker','o','MarkerSize',2,'Color','blue');
hcp = animatedline('Marker','o','MarkerSize',2,'Color','black');
ec1p = animatedline('Marker','o','MarkerSize',3,'Color','blue');
ec2p = animatedline('Marker','o','MarkerSize',3,'Color','blue');
ec3p = animatedline('Marker','o','MarkerSize',3,'Color','blue');

sprH = animatedline();
sprHJ = animatedline();
sprT = animatedline();

foot = animatedline('Marker','o');

box on;
axis equal;


last = numel(eO2x);
for i=1:last-1
    
    addpoints(exo, 0, 0);
    addpoints(exo, eO2x(i), eO2y(i));
    addpoints(exo, eO3x(i), eO3y(i));
    addpoints(exo, eO4x(i), eO4y(i));
    addpoints(tr,  eO4x(i), eO4y(i));
    
    addpoints(human, 0, 0);
    addpoints(human, hO2x(i), hO2y(i));
    addpoints(human, hO3x(i), hO3y(i));
    addpoints(human, hO4x(i), hO4y(i));
    
    addpoints(sprH, hHx(i), hHy(i)); 
    addpoints(sprH, eHx(i), eHy(i));
    addpoints(sprHJ, hHJx(i), hHJy(i)); 
    addpoints(sprHJ, eHJx(i), eHJy(i));
    addpoints(sprT, hTx(i), hTy(i)); 
    addpoints(sprT, eTx(i), eTy(i));   
    
    eC1 = [eO2x(i)/2; 
           eO2y(i)/2];

    eC2 = [(eO2x(i)+eO3x(i))/2; 
           (eO2y(i)+eO3y(i))/2 ];

    eC3 = [(eO3x(i)+eO4x(i))/2; 
           (eO3y(i)+eO4y(i))/2 ];
           
    eC = (m1*eC1 + m2*eC2 + m3*eC3) / (m1 + m2 + m3);
 
    hC1 = [hO2x(i)/2; 
           hO2y(i)/2];

    hC2 = [(hO2x(i)+hO3x(i))/2; 
           (hO2y(i)+hO3y(i))/2 ];

    hC3 = [(hO3x(i)+hO4x(i))/2; 
           (hO3y(i)+hO4y(i))/2 ];
           
    hC = (hm1*hC1 + hm2*hC2 + hm3*hC3) / (hm1 + hm2 + hm3);
    
    addpoints(ec1p, eC1(1), eC1(2));
    addpoints(ec2p, eC2(1), eC2(2));
    addpoints(ec3p, eC3(1), eC3(2));
    addpoints(ecp,  eC(1), eC(2));
    addpoints(hcp,  hC(1), hC(2));
    addpoints(foot, 0, 0);
    addpoints(foot, 0.24, 0);
    
    drawnow limitrate nocallbacks;

    clearpoints(foot);
    clearpoints(sprH);
    clearpoints(sprHJ);
    clearpoints(sprT);
    clearpoints(ec1p);
    clearpoints(ec2p);
    clearpoints(ec3p);
    clearpoints(human);
    clearpoints(exo);
end

addpoints(exo, 0, 0);
addpoints(exo, eO2x(last), eO2y(last));
addpoints(exo, eO3x(last), eO3y(last));
addpoints(exo, eO4x(last), eO4y(last));
addpoints(tr,  eO4x(last), eO4y(last));

addpoints(human, 0, 0);
addpoints(human, hO2x(last), hO2y(last));
addpoints(human, hO3x(last), hO3y(last));
addpoints(human, hO4x(last), hO4y(last));

addpoints(sprH, hHx(last), hHy(last)); 
addpoints(sprH, eHx(last), eHy(last));
addpoints(sprHJ, hHJx(last), hHJy(last)); 
addpoints(sprHJ, eHJx(last), eHJy(last));
addpoints(sprT, hTx(last), hTy(last)); 
addpoints(sprT, eTx(last), eTy(last));  

eC1 = [eO2x(last)/2; 
       eO2y(last)/2];

eC2 = [(eO2x(last)+eO3x(last))/2; 
       (eO2y(last)+eO3y(last))/2 ];

eC3 = [(eO3x(last)+eO4x(last))/2; 
       (eO3y(last)+eO4y(last))/2 ];

eC = (m1*eC1 + m2*eC2 + m3*eC3) / (m1 + m2 + m3);

addpoints(ec1p, eC1(1), eC1(2));
addpoints(ec2p, eC2(1), eC2(2));
addpoints(ec3p, eC3(1), eC3(2));
addpoints(ecp,  eC(1), eC(2));

addpoints(foot, 0, 0);
addpoints(foot, 0.24, 0);

drawnow;
