

#dataは[x,y,time]の多次元配列
def IsBranch(data):
    ERROR_MSG_VIEW_MODE=True
    counter=10;
    if(len(data)<counter):
        if(ERROR_MSG_VIEW_MODE):print("data isnot enough")
        return False;
    
    if(data[len(data)-1][0]==-1):
        data.pop(len(data)-1)
    else:    
        data.pop(0);
    error_count=0;
    for i in range(counter-1):
        if(data[i][0]==-1):error_count+=1;

    if(error_count>counter*0.4):
        if(ERROR_MSG_VIEW_MODE):print("data is UnTrustworthy",error_count,"/",counter*0.2)
        return False;
    hoge=len(data)-1

    upcount=0
    downcount=0
    place=0;
    placeX=0;
    changed=0;
    for i in range(counter-1):
        newplaceY=data[i][1];
        newplaceX=data[i][0];
        if(newplaceY==-1):continue;
        changed+=1
        if(placeX==newplaceX and newplaceY==place+50):downcount+=1;
        elif(placeX==newplaceX and newplaceY<place):downcount-=1;
        else:
            upcount+=1;
            changed-=1
        place=newplaceY;
        placeX=newplaceX;
    #print(upcount,downcount)
    if(downcount<counter*0.1):
        if(ERROR_MSG_VIEW_MODE):print("Branch isnot closing",changed,downcount,"/",counter*0.1)
        return False;

    while data[hoge][1]==-1:hoge-=1;
    if(data[hoge][1]<350):
        if(ERROR_MSG_VIEW_MODE):print("Branch is too far",changed,data[hoge][1],"/",350)
        return False;
    #if(changed>counter*0.3):
        #if(ERROR_MSG_VIEW_MODE):print("This is too damping",changed,data[len(data)-1][1],"/",350)
        #return False;
    print("★This is branch★★★★★★★★★★★★★★★★★★★",upcount,downcount,changed)
    return True;