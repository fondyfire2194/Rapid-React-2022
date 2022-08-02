import pandas as pd

from tkinter import filedialog as fd

colname =""

newList =[""]

filename = fd.askopenfilename()

##print (filename)

df = pd.read_csv(filename)


col_count = len(df.columns)

#print(col_count)

my_list = list(df)

#print(my_list)

#print("XXXXXXXXXXXXX")

x=0

#print(len(df.columns)-1)

while x<=(len(df.columns)-1):
 
     print ("X",x)
     
     str_mine = my_list[x]
     
     print("STRMINE",str_mine)
     
     if(str_mine.find('/')!=-1):

        colname=str_mine.split('/')[-1]
        
     else:

       colname = str_mine
        
     
     newList.insert(x,colname)
       
     
     x=x+1
     
     print ("XI",x)
     
newList.pop()   
 
#print(newList)

#print(len(newList))

correct_df = pd.read_csv(

  filename,
 
)
#print (correct_df)

correct_df.columns = newList

correct_df.fillna(method='ffill', inplace=True)

correct_df.fillna(method='bfill', inplace=True)

correct_df.replace({False: 0, True: 1}, inplace=True)

#print (correct_df)


index = filename.find('.csv')

filename2 = filename[:index] + 'MOD' + filename[index:]

correct_df.to_csv(filename2, index=False,header=True)



