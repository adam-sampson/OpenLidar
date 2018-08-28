library(stringr)
library(magrittr)

finfo = file.info("OL000.sph")
toread= file("OL000.sph", "rb")
alldata = readBin(toread, raw(), size=1, n = finfo$size)

#file <- file("OL000.sph","w+b")
#test <- readBin(file, "byte", n=6000)

rawToChar(alldata[48])

header <- ""
idx <- 0
while(header[length(header)]!="\n"){
  header[idx] <- rawToChar(alldata[idx])
  idx = idx + 1
}
header <- paste(header,collapse = "") %>%
  str_replace("\\r\\n","") %>%
  str_split(",") %>%
  unlist()
header

df <- data.frame(x1=integer(),x2=numeric(),x3=numeric(),x4=raw(),
                 stringsAsFactors = FALSE)
names(df) <- header  

temp <- integer()
idx2 <- 0
for(i in seq(idx,length(alldata),by = 2+4+4+2)){
  temp[idx2] <- readBin(alldata[i:(i+1)],what='integer',n=1L,size=2L,endian = "little")
  idx2 <- idx2+1
}

temp <- integer()
idx2 <- 0
for(i in seq(idx+2,length(alldata),by = 2+4+4+1)){
  temp[idx2] <- readBin(alldata[i:(i+1)],what='numeric',n=1L,size=4L)
  idx2 <- idx2+1
}
