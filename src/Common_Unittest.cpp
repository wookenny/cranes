#include <gtest/gtest.h>
#include "Common.h"

using namespace std;

TEST(Common_Methods /*Testcase Name*/,Split_Function /*Test name*/) {
    string str 		= "a,b,v ,d , ,s";
    vector<string> result1	= {"a","b","v ","d "," ","s"};
    auto split_result1 	= split(str,',');
    EXPECT_EQ(result1,split_result1);
    
    vector<string> result2	= {"a,b,v ,d , ,s"};
    EXPECT_EQ( split(str,'-'), result2 );
    
    vector<string> result3	= {"a,b,"," ,d , ,s"};
    EXPECT_EQ( split(str,'v'), result3 );   

    vector<string> result4	= {"a,b,v",",d",",",",s"};
    EXPECT_EQ( split(str,' '), result4 );   
    EXPECT_EQ( split(str), result4 );  
} 

TEST(Common_Methods /*Testcase Name*/,Interval_Function /*Test name*/) {

    
    // * "a-d"   -> {a,b,c,d} AND
    // * "[a,d]" -> {a,b,c,d}
    typedef vector<string> INTER;
    INTER ints1a =  create_interval("-3-6"); 
    INTER ints1b = {"-3","-2","-1","0","1","2","3","4","5","6"};
    EXPECT_EQ( ints1a, ints1b);  
    EXPECT_EQ( create_interval("[-3,6]"), ints1b);      
    
    INTER ints2a =  create_interval("-4--1"); 
    INTER ints2b = {"-4","-3","-2","-1"};
    EXPECT_EQ( ints2a, ints2b);
    EXPECT_EQ( create_interval("[-4,-1]"), ints2b);   
       
    INTER ints3a =  create_interval("4--1"); 
    INTER ints3b = {};
    EXPECT_EQ( ints3a, ints3b);
    EXPECT_EQ( create_interval("[4,-1]"), ints3b);  
    
    INTER ints4a =  create_interval("-4--5"); 
    INTER ints4b = {};
    EXPECT_EQ( ints4a, ints4b);
    EXPECT_EQ( create_interval("[-4,-5]"), ints4b);  
        
    INTER ints5a =  create_interval("4-2"); 
    INTER ints5b = {};
    EXPECT_EQ( ints5a, ints5b);
    EXPECT_EQ( create_interval("[4,2]"), ints5b);      
      
    
    // * "a"     -> {a}
    INTER int6 = {"213"}; INTER int7 = {"2"};
    EXPECT_EQ(int6, create_interval("213"));
    EXPECT_EQ(int7, create_interval("2"));
    
    
    // * "a,b,d" -> {a,b,d}
    INTER int9 = {"2","-5","7","0"};
    EXPECT_EQ(int9, create_interval("2,-5,7,0"));
} 
