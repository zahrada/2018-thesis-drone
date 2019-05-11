
clc
clear all 

Matrix = load('data_2018_11_16_9_44.m');

eps = 0.0000000001;
n = length(Matrix);
index = 1;

heading_prev = 0;
pitch_prev = 0;
roll_prev = 0;
north_prev = 0;

for j= 1:n
sum_heading = 0;
sum_pitch = 0;
sum_roll = 0;
sum_north = 0;

v_sum_heading = 0;
v_sum_pitch = 0;
v_sum_roll = 0;
v_sum_north = 0;

heading = [0 0];
pitch = [0 0];
roll = [0 0];
north = [0 0];

%Calculete sum of each data for average
  for i = index:n
      sum_heading = sum_heading + Matrix(i,1);
      sum_pitch = sum_pitch + Matrix(i,2);
      sum_roll = sum_roll + Matrix(i,3);
      sum_north = sum_north + Matrix(i,4);
      j(i) = i;
  end
  % average od each date
heading(1,1) = sum_heading/(n-index);
pitch(1,1) = sum_pitch/(n-index);
roll(1,1) = sum_roll/(n-index);
north(1,1) = sum_north/(n-index);

% rozptyl
  for i = index:n
      v_sum_heading = v_sum_heading + (heading(1,1) - Matrix(i,1))*(heading(1,1) - Matrix(i,1));
      v_sum_pitch =  v_sum_pitch + (pitch(1,1) - Matrix(i,2))*(pitch(1,1) - Matrix(i,2));
      v_sum_roll = v_sum_roll + (roll(1,1) - Matrix(i,3))*(roll(1,1) - Matrix(i,3));
      v_sum_north = v_sum_north + (north(1,1) - Matrix(i,4))*(north(1,1) - Matrix(i,4));
  end

  heading(1,2) = v_sum_heading / (n-index);
  pitch(1,2) =  v_sum_pitch / (n-index);
  roll(1,2) = v_sum_roll / (n-index);
  north(1,2) = v_sum_north / (n-index);
  
  if abs(pitch_prev-pitch(1,2))>eps && abs(roll_prev-roll(1,2))>eps
      index = index + 1;
  else
      break
  end
      
  heading_prev = heading(1,2);
  pitch_prev = pitch(1,2);
  roll_prev = roll(1,2);
  north_prev = north(1,2);
end
  

%--------------------------------------------------------------------------
sum_heading = 0;
sum_pitch = 0;
sum_roll = 0;
sum_north = 0;
v_sum_heading = 0;
v_sum_pitch = 0;
v_sum_roll = 0;
v_sum_north = 0;
heading = [0 0];
pitch = [0 0];
roll = [0 0];
north = [0 0];

%Calculete sum of each data for average
  for i = index:n
      sum_heading = sum_heading + Matrix(i,1);
      sum_pitch = sum_pitch + Matrix(i,2);
      sum_roll = sum_roll + Matrix(i,3);
      sum_north = sum_north + Matrix(i,4);
      j(i) = i;
  end
  % average od each date
heading(1,1) = sum_heading/(n-index);
pitch(1,1) = sum_pitch/(n-index);
roll(1,1) = sum_roll/(n-index);
north(1,1) = sum_north/(n-index);

% rozptyl
  for i = index:n
      v_sum_heading = v_sum_heading + (heading(1,1) - Matrix(i,1))*(heading(1,1) - Matrix(i,1));
      v_sum_pitch =  v_sum_pitch + (pitch(1,1) - Matrix(i,2))*(pitch(1,1) - Matrix(i,2));
      v_sum_roll = v_sum_roll + (roll(1,1) - Matrix(i,3))*(roll(1,1) - Matrix(i,3));
      v_sum_north = v_sum_north + (north(1,1) - Matrix(i,4))*(north(1,1) - Matrix(i,4));
  end

  heading(1,2) = v_sum_heading / (n-index);
  pitch(1,2) =  v_sum_pitch / (n-index);
  roll(1,2) = v_sum_roll / (n-index);
  north(1,2) = v_sum_north / (n-index);

  fprintf('Start from %d\n',index);
  fprintf('Heading is %f %f\n',heading);
  fprintf('Pitch is %f %f\n',pitch);
  fprintf('Roll is %f %f\n',roll);
  fprintf('North is %f %f\n',north);