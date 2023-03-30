function f_min=testfunc(R_T)
global POINT;
global POINT_INIT;
global W;
d=size(R_T,1);
for i=1:d
    R(:,:,i)=eul2rotm([R_T(i,6),R_T(i,5),R_T(i,4)]);
    T(:,i)=[R_T(i,1),R_T(i,2),R_T(i,3)]';
end
for i=1:d
    f(:,:,i)=POINT-(R(:,:,i)*POINT_INIT+repmat(T(:,i),[1,size(POINT_INIT,2)]));
end
for i=1:d
    f_min(i)=sum(f(:,:,i).*f(:,:,i)*W);
end
end


    