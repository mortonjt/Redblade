line0.5data = read.table("/home/jamie/Downloads/angular_test3/trimmed_line0.5.csv",header=FALSE)
cmdVels = line0.5data$V1
ImuVels = line0.5data$V2

Reduced = lm(ImuVels~cmdVels)

plot(cmdVels,ImuVels)
abline(Reduced)

Reduced = lm(cmdVels~ImuVels)
summary(Reduced)

plot(Reduced$fitted,rstandard(Reduced),xlab="Fitted Values",ylab="Standardized Residuals")
qqnorm(rstandard(Reduced))
qqline(rstandard(Reduced))



line1.0data = read.table("/home/jamie/Downloads/angular_test3/trimmed_line1.0.csv",header=FALSE)
cmdVels = line1.0data$V1
ImuVels = line1.0data$V2

Reduced = lm(ImuVels~cmdVels)

plot(cmdVels,ImuVels)
abline(Reduced)

Reduced = lm(cmdVels~ImuVels)
summary(Reduced)

plot(Reduced$fitted,rstandard(Reduced),xlab="Fitted Values",ylab="Standardized Residuals")
qqnorm(rstandard(Reduced))
qqline(rstandard(Reduced))
