info=imaqhwinfo;
win_info = imaqhwinfo('winvideo');
win_info.DeviceInfo(2);
inputCamera = videoinput('winvideo',2,'MJPG_1024x576');
preview(inputCamera)

camera = cv.VideoCapture();
for j=1:100
img= camera.read; % ����һ֡��Ƶ
img=imresize(img,0.5); % ������Сһ��
imshow(img); % ��ʾͼ��
pause(0.001); % ��ͣ0.001��
end
camera.delete; % �ر�����ͷ

