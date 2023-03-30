info=imaqhwinfo;
win_info = imaqhwinfo('winvideo');
win_info.DeviceInfo(2);
inputCamera = videoinput('winvideo',2,'MJPG_1024x576');
preview(inputCamera)

camera = cv.VideoCapture();
for j=1:100
img= camera.read; % 读出一帧视频
img=imresize(img,0.5); % 窗口缩小一半
imshow(img); % 显示图像
pause(0.001); % 暂停0.001秒
end
camera.delete; % 关闭摄像头

