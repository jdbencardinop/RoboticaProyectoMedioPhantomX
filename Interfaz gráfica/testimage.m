%% Test Image

rosshutdown
setenv('ROS_MASTER_URI', 'http://25.12.80.0:11311')
rosinit

%%Create ROS Node
%node = rosmatlab.node('/matlab_image', 'http://25.12.80.0:11311');
%%Create subscriber
%% Suscribir
subscriber_imageGrabber = rossubscriber('/rrbot/camera1/image_raw', @processImage);
global imagen

function processImage(~,message)
      A = typecast(message.Data, 'uint8');
      height = message.Height;
      width = message.Width;
      A = A';
      A = reshape(A, [height width 3]);
      imagen = A;
      figure();
      imshow(A);
      display(height,'altura')
      display(width, 'ancho')
      display(size(A),'tam')
      pause;
end