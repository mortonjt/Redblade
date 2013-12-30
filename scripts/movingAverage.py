

def movingAverageWindow(windows):
    avgs = sum([x for x in windows])/len(windows)
    return avgs

def weightedMovingAverageWindow(windows):
    n = len(windows)
    avgs = sum([x*(n-i) for x in windows])/sum(range(0,n))
    return avgs

def movingAverage(msgs,order):
    windows = zip(*[msgs[i:-(order-i)] for i in range(0,order)] )
    avgs = map(movingAverageWindow,windows)
    return avgs

def weightedMovingAverage(msgs,order):
    windows = zip([ msgs[i:-(order-i)] for i in range(0,order) ])
    avgs = map(weighedMovingAverageWindow,windows)
    return avgs

def exponentialMovingAverage(msgs,alpha):
    avgs = [msgs[0]]
    for i in range(1,len(msgs)):
        avgs.append( alpha*msgs[i]+(1-alpha)*avgs[-1])
    return avgs
