function newX = horizontalMove(rWheelV,lWheelV,theta)
    enFrequency = 10; %(Hz)
    enTime = 1/enFrequency; %(s)

    newX = ((rWheelV/2)+(lWheelV/2)) * cos(theta) * enTime;

end