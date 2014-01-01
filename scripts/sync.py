import numpy
import bisect

"""
Used for filtering encoder readings with gps reference data
"""
def filterRawInput(refTimes,rawTimes,rawInputs,threshold):
    filteredInputs,filteredTimes = [],[]
    refTimes.sort()
    for i in xrange(0,len(rawTimes)):
        j = bisect.bisect_left(refTimes,rawTimes[i])
        if j<len(refTimes) and abs(refTimes[j]-rawTimes[i]) < threshold:
            filteredInputs.append(rawInputs[i])
            filteredTimes.append(rawTimes[i])
    return filteredInputs,filteredTimes

"""
Matches raw input with reference input
"""
def matchInput(refTimes,refInput,rawTimes,rawInput,threshold):
    refTimeBins      = [refTimes[i]-threshold
                        for i in xrange(0,len(refTimes))]
    matchedRaw,times = filterRawInput(refTimes,rawTimes,rawInput,threshold)
    indexes          = numpy.digitize(times,refTimeBins)[:-1]
    indexes          = [x for x in indexes if x<len(refInput)]

    matchedRaw       = [matchedRaw[i] for i in xrange(0,len(indexes))]
    matchedRef       = [refInput[indexes[i]] for i in xrange(0,len(indexes))]
    return matchedRef,matchedRaw
