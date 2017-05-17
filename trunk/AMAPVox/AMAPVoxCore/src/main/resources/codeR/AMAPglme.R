
rm(list=ls())
library(lme4)

Jcode <- function(){
    dt = data.frame(ijk=ijk, prop=prop, trials=trials)
    mod1=glmer(prop~1|ijk, weights=trials,family=binomial(link="logit"), data=dt)
    return(predict(mod1,type="response"))
}