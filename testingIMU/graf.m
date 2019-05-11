clc
clear all 

Matrix = load('data_2018_11_16_9_44.m');

index = zeros(length(Matrix),1);
for i=1:length(Matrix)
    index(i) = i;
end
plot(index, Matrix(:,2),'red',index, Matrix(:,3),'blue');
xlabel('Index (10us)')
ylabel('stupnì')
legend('Pitch','Roll')