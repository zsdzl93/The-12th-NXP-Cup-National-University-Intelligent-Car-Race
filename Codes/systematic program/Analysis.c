#include "common.h"
#include "include.h"
#include "self_include.h"


void dataAnalysis()        //Encoder pulse acquisition function
{
	middleLine();
	middleline_37_36();
}


void middleLine()  //Middle line, road recognition
{
	int i, middleline=middleline_start, j, ROW=59, COL=79;
	int Left_Black=middleline_start;
	int Right_Black=middleline_start;

	//Initial drawing midline method non-ring and cross road
	for(i=ROW;i>=outBoundRow;i--)
	{
		for(j= middleline;j>=0;j--)  // Search from the middle to the left to find black spots
		{
			if(img[i*80+j-1]==0xff&& img[i*80+j-1-1]==0x00)
			{
				left_Black=j;
				break;
			}
			else Left_Black=0;  // No black spots on the left
		}
		for(j= middleline;j<=COL;j++)	// Search from the middle to the right to find black spots
		{
			if(img[i*80+j-1-1]==0xff && img[i*80+j-1]==0x00)
			{
				Right_Black=j;
				break;
			}
			else Right_Black=COL;   //No black spots on the right
		}

		middleline=(Left_Black + Right_Black)/2;
		mid[i]=middleline;
		right[i]=Left_Black;
		right[i]=Right_Black;
	}
  
	path_scan();
	loseBoundOnOneSide();
	cross_scan();
	ringRecognition();
	exceptionHandling();
	downhill();
	stop();
	obstacleRecognition();

	//Determine the starting point of the next midline search
	if( leftTriangleFlag==1&&now_outBoundRow>10) middleline_start=10;
	else if(rightTriangleFlag==1&&now_outBoundRow>10) middleline_start=70;
	else middleline_start=40;
}


/*////////////////////////////////////////////////////////////////////
        middleline_37_36 function
Get the position of the center line corresponding to the current control line as the input of the PID
/////////////////////////////////////////////////////////////////////*/
void   middleline_37_36()    // filter
{
	int sum;
	sum=mid[ControlRow+3]+mid[ControlRow+2]+mid[ControlRow+1]+mid[ControlRow]+mid[ControlRow-1]+mid[ControlRow-2]+mid[ControlRow-3];
	AverageMiddleline=sum/7; //mid[ControlRow]
}

///////////////////////////////////////////////////////////////////////
//   *       path_scan function
//   *       Determine the current state
////////////////////////////////////////////////////////////////////////

void path_scan()
{  
	int i,j;
	outBoundRow=2;//control_line1;
	oldoutBoundRow=59;
	smallestColumn=20;

   //Small triangle recognition, used for the judgment of the starting point of the next midline search
	for(i=59;i>=6;i--)
	{
		leftTurn_flag1+=left[i]<=2&&right[i]<30;
		rightTurn_flag1+=left[i]>=60&&right[i]>=77;
		if(leftTurn_flag1>=1||rightTurn_flag1>=1) break;
	}

    if(leftTurn_flag1>0)
    {
		leftSmallTriangleflag=1;
		leftTurn_flag1=0;
		leftTriangleFlag=1;
		rightTriangleFlag=0;
    }
      
    if(rightTurn_flag1>0)
    {
		rightSmallTriangleflag=1;
		rightTurn_flag1=0;
		leftTriangleFlag=0;
		rightTriangleFlag=1;
    }

	for(j=0;j<=79;j++) outBoundRecord[j]=0;
    
	//search for outbound row and upper bound.
	for(j=0;j<=79;j++) // Outbound line detection from 59 column to 0 column
	{
		for(i=59;i>=10;i--)        // i=50->10 for direction control£¬i=59->50 for exception handling
		{
			if( img[i*80+j]==0x00)  // Look for the first black dot from the bottom up
			{
				outBoundRecord[j]=i;
				outBoundRow=i<=oldoutBoundRow?i:oldoutBoundRow;   // Take the minimum value, e.g. the highest black point
				if(i<oldoutBoundRow)
				smallestColumn=j;
				break;
			}
			else outBoundRecord[j]=10; // 10, if black point is not found.
		}
		oldoutBoundRow=outBoundRow;
	}
    now_outBoundRow=outBoundRow;
}



/****************Judging whether there is bound losing on one side****************/
void loseBoundOnOneSide()
{
	int i;
	loseRightBound_flag=0;
	loseLeftBound_flag=0;
	loseLeftBound=0;
	loseRightBound=0;

	if (ringDelay>0) ringDelay--;

	if (ringDelay<=1)
    {
		for(i=outBoundRow;i<=50;i++)
		{
			if(left[i]<=2) loseLeftBound++;
			if(right[i]>=78) loseRightBound++;

			if(loseRightBound>=15)
			{
				loseRightBound_flag=1;
				loseLeftBound_flag=0;
				break;
			}
			if(loseLeftBound>=15)
			{
				loseLeftBound_flag=1;
				loseRightBound_flag=0;
				break;
			}
		}
	if( loseRightBound_flag==1 && loseLeftBound_flag!=1 && crossflag==0)
	{
		fillRightBound=1;
		if( right[59]>78 ) right[59]=79;
		// Compensate line for turning
		for(i=58;i>=6;i--)
		{
			if(right[i]>78 && i<=57 )
			{
				mid[i] =  left[i] + ( mid[i+1]-left[i+1] ) + 0 ;
				if( mid[i]>78 ) mid[i]=79;
			}
		}
	}
	else fillRightBound=0;
     
	if( loseLeftBound_flag==1&&loseRightBound_flag!=1 &&crossflag==0)
	{
		fillLeftBound=1;
		if( left[59]<2 ) left[59]=0 ;

		for(i=58;i>=6;i--)
		{
			if( left[i]<3 && i<=57 )
			{
				mid[i] = right[i] - ( right[i+1]-mid[i+1] ) + 0.1 ;//  -
				if( mid[i]<2 ) mid[i]=0;
			}
		}
	}
	else fillLeftBound=0;
    }
}


//******************************* obstacle recognition ****************************/
void obstacleRecognition()
{
	int i,j,obstacle_middleLine,M[3]={0};
	obstacleflag=0,narrowBottomflag=0;
	leftObstacleflag=0,rightObstacleflag=0;
	leftPosLeap=0;	//left positive leap
	leftNegLeap=0;	//left negative leap
	rightPosLeap=0;	//right positive leap
	rightNegLeap=0;	//right negative leap
  
	// if entered ring or crossroad, then no judgement for the obstacle
	if(leftEmptyflag==1||rightEmptyflag==1||ringflag==1) cross_ring_obstacleDelay=80;
	else if(ringflag==0&&leftEmptyflag==0&&rightEmptyflag==0)
	{
		if(cross_ring_obstacleDelay>0) cross_ring_obstacleDelay--;
		else if(stop_obstacleDelay>0) stop_obstacleDelay--;
		else if(cross_ring_obstacleDelay==0 && stop_obstacleDelay==0)
		{
			for(i=58;i>=20;i--)//compute left and right changes
			{
				if((img[80*i+left[i]+2]==0xff)&&(img[80*(i+1)+left[i+1]+2]==0xff)&&(img[80*(i-1)+left[i-1]+2]==0xff))
					leftDerivative[i]=left[i]-left[i+1];
				else leftDerivative[i]=0;

				if((img[80*i+right[i]-2]==0xff)&&(img[80*(i+1)+right[i+1]-2]==0xff)&&(img[80*(i-1)+right[i-1]-2]==0xff))
					rightDerivative[i]=right[i]-right[i+1];
				else rightDerivative[i]=0;
			}
			// search for left positive leap to recognize obstacle on left side
			for(i=58;i>=10;i--)
			{
				if(leftDerivative[i]>6&&leftDerivative[i]<25)   //8
				{
					leftPosLeap=i;
					for(j=i;j>=10;j--)
					{
						if(leftDerivative[j]<=-6)      //-6     j
						{
							leftNegLeap=j;
							if(i-j>3)
							{
								leftObstacleflag=1;
								break;
							}
						}
					}
				}
			}
			// search for right positive leap to recognize obstacle on right side
			for(i=58;i>=10;i--)
			{
				if(rightDerivative[i]<-6&&rightDerivative[i]>-25)
				{
					rightNegLeap=i;
					for(j=i;j>=10;j--)
					{
						if(rightDerivative[j]>=6)
						{
							rightPosLeap=j;
							if(i-j>3)
							{
								rightObstacleflag=1;
								break;
							}
						}
					}
				}
			}
			// compensate the line for the left obstacle
			if(leftObstacleflag==1)
			{
				M[0] = (left[leftPosLeap-2]+right[leftPosLeap-2])/2;
				M[1] = (left[leftPosLeap-1]+right[leftPosLeap-1])/2;
				M[2] = (left[leftPosLeap]+right[leftPosLeap])/2;
				obstacle_middleLine = (M[0]+M[1]+M[2])/3;
				for(i=35;i<=45;i++)// Using the lower edge of the obstacle to get a good path, and the deviation is also larger
				{
					mid[i]=obstacle_middleLine;
				}
			}
			// compensate the line for the right obstacle
			if(rightObstacleflag==1)
			{
				M[0] = (left[rightNegLeap-2]+right[rightNegLeap-2])/2;
				M[1] = (left[rightNegLeap-1]+right[rightNegLeap-1])/2;
				M[2] = (left[rightNegLeap]+right[rightNegLeap])/2;
				obstacle_middleLine = (M[0]+M[1]+M[2])/3;
				for(i=35;i<=45;i++)
				{
					mid[i]=obstacle_middleLine;
				}
			}
			if(leftObstacleflag==1||rightObstacleflag==1)
			{
				obstacleflag=leftObstacleflag||rightObstacleflag;
				obstacleDelay=100;
//				led( LED2, LED_ON );
			}
			else
			{
				obstacleflag=0;
//				led( LED2, LED_OFF );
			}
			//See if the bottom edge is very narrow (used in passing by obstacles)
			if(obstacleDelay>0&&(right[59]-left[59]<45)&&(right[58]-left[58]<45)&&(right[57]-left[57]<45)&&(right[56]-left[56])<45&&(right[55]-left[55])<45)
			{
				narrowBottomflag = 1;  //narrow bottom flag
				narrowBottomDelay = 60;
				M[0] = (left[58]+right[58])/2;
				M[1] = (left[57]+right[57])/2;
				M[2] = (left[56]+right[56])/2;
				obstacle_middleLine = (M[0]+M[1]+M[2])/3;
				for(i=35;i<=45;i++)
				{
					mid[i]=obstacle_middleLine;
				}
				led( LED3, LED_ON );
			}
			else
			{
				narrowBottomflag=0;
				led( LED3, LED_OFF );
			}
		}

		if(obstacleDelay>0) obstacleDelay--;
		else obstacleDelay=0;

		if(narrowBottomDelay>0) narrowBottomDelay--;
		else narrowBottomDelay=0;
	}
}



/***************** Crossroad recognition ( judge right/left space for ring & crossroad )****************/
void cross_scan()
{   
	int i,j,L=0,R=79 ,COL=79;
	// left empty?
	for(i=50;i>(outBoundRow+10);i--)
	{
		if( (left[i]<1) && (left[i-1]<1) && (left[i-2]<1) && (left[i-3]<1)&&left[i-4]<1&&left[i-5]<1 )
		{
			leftEmptyflag=1;
			leftEmptyRow=i;
			break;
		}

		leftEmptyflag=0;
	}
      
	// right empty?
	for(j=50;j>(outBoundRow+10);j--)
	{
		if( (right[j]>78) && (right[j-1]>78) && (right[j-2]>78)&& (right[j-3]>78)&&right[j-4]>78 &&right[j-5])
		{
			rightEmptyflag=1;
			break;
		}

		rightEmptyflag=0;
	}
      
	// //when both sides are empty, let crossflag=1, otherwise 0
	if( (leftEmptyflag==1) && (rightEmptyflag==1) )
		crossflag=1;
	else crossflag=0;
	// Middle line drawing method for ring and crossroad:
	// search for left and right borders from the outbound line, the smallest column
	if(crossflag==1&&outBoundRecord[40]<15)
	{
		int outBoundRow_start;

		if(outBoundRow<20) outBoundRow_start=20;
		else outBoundRow_start=outBoundRow;

		for(i=outBoundRow_start;i<=59;i++)
		{
			for(j= smallestColumn;j>=0;j--)		// Search from the middle to the left to find black spots
			{
				if(img[i*80+j-1]==0xff&& img[i*80+j-1-1]==0x00)
				{
					Left_Black=j;
					break;
				}
				else Left_Black=0;		// No black spots on the left
			}

			for(j= smallestColumn;j<=COL;j++)		// Search from the middle to the right to find black spots
			{
				if(img[i*80+j-1-1]==0xff && img[i*80+j-1]==0x00)
				{
					Right_Black=j;
					break;
				}
				else Right_Black=COL;   // No black spots on the left
			}

			left[i]=Left_Black;
			right[i]=Right_Black;
			L = L>=left[i]?L:left[i];      // Take bigger one
			R = R<=right[i]?R:right[i];    // Take smaller one
		}
		for(i=outBoundRow;i<55;i++)
		{
			mid[i]=(L+R)/2;
		}
	}
}

/********************** ring recognition function ************************/
void ringRecognition()
{
	int i,j;
	//  Find black and white border between 20 and 30 lines
	for (i=25;i>15;i--)               //20-30
	{
		for(j= 40;j>=0;j--)		// Search from the middle to the left to find black spots
		{
			if(img[i*80+j-1]!=0xff && img[i*80+j-1-1]!=0x00)
			{
				Left_B2W=j;
				break;
			}
			else Left_B2W=0;	// No black spots on the left
		}
		for(j= 40;j<=80;j++)	// Search from the middle to the right to find black spots
		{
			if(img[i*80+j-1-1]!=0xff && img[i*80+j-1]!=0x00)
			{
				Right_B2W=j;
				break;
			}
			else Right_B2W=80;   // No black spots on the left
		}

		if( (Right_B2W-Left_B2W>20)&&(Right_B2W-Left_B2W<70))    //60
		{
//			circle_length = Right_B2W-Left_B2W;
			break;
		}
	}
  
	//   The black block width in front meets the requirements of 20-50  &  Black and white border in front & both sides empty
	for(i=45;i>=15;i--)
	{
		if( (Right_B2W-Left_B2W>20)&&(Right_B2W-Left_B2W<70)&&(img[80*i+40]!=0x00)&&(img[80*(i-1)+40]!=0xff)&&(crossflag==1)&&(outBoundRecord[40]>=15)  )
		{
			ringflag=1;
//			ringDelay=50;
			ringEnterflag=1;
			led( LED2, LED_ON );
			led( LED3, LED_ON );
			break;
		}
		else
		{
			ringflag=0;
			led( LED2, LED_OFF );
			led( LED3, LED_OFF );
		}
	}
}



/********************** exception handling function ************************/
// Control or protect the car after it is about to rush out of the track or has already rushed out of the track.
void exceptionHandling()
{
	if(outBoundRow>55) // If the car rushed out of the track, then motors output are 0.
		exceptionflag1=1; // Do not clear this flag again
	else if(outBoundRow>50&&(leftTriangleFlag==1||rightTriangleFlag==0)) // might be saved by small triangle when it is about to be out of bound.
		exceptionflag2=1;
	else
		exceptionflag2=0;
}


/********************** Stop car function ************************/
// Identification stop line
void stop()
{
	int j,k,M[3],stop_middleLine,stopline=0;
	float i;

	if(stoptimes==0||t>10000)
	{
		for(i=10.0;i<=58.0;i++)
		{
			B2H=0,H2B=0;
			// Record the number of times the white has turned black
			for( k=i,j=-0.25*i+25;j<=0.25*i+55;j++)
			{
				if( img[k*80+j]==0xff&&img[k*80+j+1]==0x00)
					B2H++;
			}
			// Record the number of times the black has turned white
			for(k=i,j=-0.25*i+25;j<=0.25*i+55;j++)
			{
				if( img[k*80+j]==0x00&&img[k*80+j+1]==0xff)
					H2B++;
			}
			if(B2H>=5&&H2B>=5)
			{
				stopline++;
				upperBound=i<oldupperBound?i:oldupperBound;
				lowerBound=i>oldlowerBound?i:oldlowerBound;
			}
		}
		if(stopDelay==0&&stopline>=3)
		{
			stoptimes++;
			stopDelay=10;
		}

		if(stoptimes==1) stop_obstacleDelay=45;

		if(stoptimes>1&&stopDelay==0) stopflag=1;
	}
  
	if(stopDelay>0)
	{
		stopDelay--;
		if(lowerBound<45)
		{
			M[0] = (left[lowerBound+5]+right[lowerBound+5])/2;
			M[1] = (left[lowerBound+6]+right[lowerBound+6])/2;
			M[2] = (left[lowerBound+7]+right[lowerBound+7])/2;
			stop_middleLine = (M[0]+M[1]+M[2])/3;
		}
		for(k=35;k<=45;k++)
		{
			mid[k]=stop_middleLine-3;
		}
	}
	else stopDelay=0;
}


/********************** ramp process function for Laboratory ************************/
void downhill()
{
	int i,ramp=0,bothSideEmptyRamp=0;
	if(downhillDelay>0) downhillDelay--;
	// Prevent misjudgment as a crossroad
	if(leftEmptyflag==1||rightEmptyflag==1)
		bothSideEmptyRamp=1;
	else
	{
		for(i=15;i<=29;i++)
		{
			if(left[i]<left[55]||right[i]>right[55])
			{
				bothSideEmptyRamp=1;
				break;
			}
		}
	}

	if(rampSet==1&&outBoundRow<12&&obstacleflag==0&&stopDelay==0&&bothSideEmptyRamp==0)
	{
		//20 - 35 rows Derivate and filter to obtain approximate fitting slope
		// left slope
		leftRampBound[18]=0;
		for(i=15;i<=21;i++)
		{
			leftRampBound[18]+=left[i];
		}
		leftRampBound[18]=leftRampBound[18]/7;

		leftRampBound[26]=0;
		for(i=23;i<=29;i++)
		{
			leftRampBound[26]+=left[i];
		}
		leftRampBound[26]=leftRampBound[26]/7;
		leftSlope=(leftRampBound[18]-leftRampBound[26])/8;
		// right slope
		rightRampBound[18]=0;
		for(i=15;i<=21;i++)
		{
			rightRampBound[18]+=right[i];
		}
		rightRampBound[18]=rightRampBound[18]/7;

		rightRampBound[26]=0;
		for(i=23;i<=29;i++)
		{
			rightRampBound[26]+=right[i];
		}
		rightRampBound[26]=rightRampBound[26]/7;
		rightSlope=(rightRampBound[26]-rightRampBound[18])/8;


		// Find the fitting width below
		for(i=40;i<55;i++)
		{
			leftRampBound[i]=left[22]-leftSlope*(i-22);
			rightRampBound[i]=right[22]+rightSlope*(i-22);
		}
		// Compare the actual width and fit width of the 10 rows below
		for(i=40;i<55;i++)
		{
			if((right[i]-rightRampBound[i])>=1&&(leftRampBound[i]-left[i])>=1)
				ramp++;
		}
		if(ramp>=12&&rightSlope>0&&leftSlope>0)
		{
			downhillflag=1;
			downhillDelay=30;
//			led( LED2, LED_ON );
		}
	}
	else
	{
		downhillflag=0;
//		led( LED2, LED_OFF );
	}
}





























