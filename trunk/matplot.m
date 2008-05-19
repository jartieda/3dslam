clear

load feat.txt
figure(1)
plot3(feat(:,1), feat(:,2), feat(:,3),'.');
hold on
load cam.txt
plot3(cam(:,1),cam(:,2),cam(:,3),'r');
axis equal
grid on
figure(2)
s=size(feat,1);
hold off
plot3(-feat(s-40:s,1), feat(s-40:s,2), -feat(s-40:s,3),'.');
hold on
load cam.txt
plot3(-cam(:,1),cam(:,2),-cam(:,3),'r');
axis equal
grid on
hold off
% figure(3)
% load disp19.txt
% load disp10.txt
% load disp20.txt
% load disp30.txt
% load disp40.txt
% load disp50.txt
% load disp60.txt
% load disp70.txt
% plot3(disp10(:,1),disp10(:,2),disp10(:,3),'b.');
% hold on
% plot3(disp20(:,1),disp20(:,2),disp20(:,3),'g.');
% plot3(disp30(:,1),disp30(:,2),disp30(:,3),'r.');
% plot3(disp40(:,1),disp40(:,2),disp40(:,3),'c.');
% %plot3(disp50(:,1),disp50(:,2),disp50(:,3),'m.');
% %plot3(disp60(:,1),disp60(:,2),disp60(:,3),'k.');
% %   plot3(disp70(:,1),disp70(:,2),disp70(:,3),'r.');
% %plot3(cam(:,1),cam(:,2),cam(:,3),'g');
% axis equal
% grid on
% hold off
figure (4)
subplot(2,2,1)
load disp0.txt
load disp1.txt
load disp2.txt
load disp3.txt
plot3(disp0(:,3),disp0(:,1),disp0(:,2),'b.');
axis equal
grid on
subplot(2,2,2)
plot3(disp1(:,3),disp1(:,1),disp1(:,2),'b.');
axis equal
grid on
subplot(2,2,3)
plot3(disp2(:,3),disp2(:,1),disp2(:,2),'b.');
axis equal
grid on
subplot(2,2,4)
plot3(disp3(:,3),disp3(:,1),disp3(:,2),'b.');
axis equal
grid on
figure (5)
subplot(2,2,1)
d1=load ('disp40.txt')
d2=load ('disp43.txt')
d3=load ('disp46.txt')
d4=load ('disp49.txt')
plot3(d1(:,3),d1(:,1),d1(:,2),'b.');
axis equal
grid on
subplot(2,2,2)
plot3(d2(:,3),d2(:,1),d2(:,2),'b.');
axis equal
grid on
subplot(2,2,3)
plot3(d3(:,3),d3(:,1),d3(:,2),'b.');
axis equal
grid on
subplot(2,2,4)
plot3(d4(:,3),d4(:,1),d4(:,2),'b.');
axis equal
grid on
