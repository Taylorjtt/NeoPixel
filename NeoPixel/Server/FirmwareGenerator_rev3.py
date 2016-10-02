#!/usr/bin/python
import os
import datetime
i = datetime.datetime.now()
 
#turns input of an array of chars into an array of integers from every two chars interpreted as hex values
def createHexIntValues(charValues):
	i = 0
	hexValues = []
	while i < len(charValues):
		#end condition is no more values
		value = '0x' + charValues[i] + charValues[i+1]
		i = i+2
		hexValues.append(int(value,16))
	return hexValues
	#check values
	#print(hexValues)

def createCheckSum(hexValues):
	Csum = 0
	i = 0
	while i < len(hexValues):
		Csum += hexValues[i]
		i = i+1
	#get_bin = lambda x: x >= 0 and str(bin(x))[2:] or "-" + str(bin(x))[3:]
	#print(hex(Csum))
	return hex(Csum)

def make2Bytes(string):
	if(len(string)<2):
		string = '0' + string
	return string


try:
    #get input
	#inputfilename = raw_input("input the binary filename: ")
	inputfilename = "MyocycleHome.txt"
	#print ("Hello")
	#print ("Current date & time = %s" % i)
	#print ("Current year = %s" %i.year)
	year = str(i.year-2000)
	month = str(i.month)
	version = str(i.day)
	unique = hex((i.hour*12)+(i.minute//5))
	stringunique = str(unique)
	priority = stringunique[2:]

	#create file name
	binFile = open( inputfilename ,"r" )
	outputFileID = ''
	year = make2Bytes(year)
	outputFileID += year
	month = make2Bytes(month)
	outputFileID += month
	version = make2Bytes(version)
	outputFileID += version
	priority = make2Bytes(priority)
	outputFileID += priority
	outputFileName = outputFileID + '.txt'
	outputFile = open(outputFileName, 'w', 0)

	binValues = []
	binHeader = []
	binZeros  = []
	#insert the version number
	binHeader.append('=')
	binHeader.append(outputFileID)

	#add hex data values to binValues array
	while True:
		#read value from input file
		c = binFile.read(1)
		#if at end of file
		if not c:
			break
		#if c is a wanted value
		if c != ' ' and c != '\n' and c != '' and c != '':
			binValues.append(c)
	#convert from char to integers
	HexIntValues = createHexIntValues(binValues)
	#Create the checksum
	checksum = createCheckSum(HexIntValues)
	#put checksum into the header
	binHeader.append('#')
	Final_checksum = '000000000' + checksum[2:]
	binHeader.append(Final_checksum[-8:])                           #
	#put number of bytes into the header					
	binHeader.append('?')									#
	Number_bytes = str(hex(len(HexIntValues)))
	Final_number_bytes = '000000000' + Number_bytes[2:]
	binHeader.append(Final_number_bytes[-8:])			#
#	binHeader.append(str(len(HexIntValues)))			#  Original
	binHeader.append('<')
	binZeros.append('/000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000')

	#write header values from array to the output file
	for val in binHeader:
		outputFile.write(val)
	#write the data values from array into the output file
	for val in binValues:
		outputFile.write(val)
	#writes Zeros at the end into the output file
	for val in binZeros:
		outputFile.write(val)
	outputFile.flush()
	os.fsync(outputFile)
	outputFile.close()
	#print(binValues)

	#input("enter anything to close")


except Exception as ex:
    print ex
    raw_input()
