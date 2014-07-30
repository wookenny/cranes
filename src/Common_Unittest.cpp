#include <gtest/gtest.h>
#include <vector>
#include <string>
#include <set>
#include "./Common.h"

using std::vector;
using std::string;

TEST(Common_Methods /*Testcase Name*/,Split_Function /*Test name*/) {
    string str = "a,b,v ,d , ,s";
    vector<string> result1 = {"a", "b", "v ", "d ", " ", "s"};
    auto split_result1 = split(str, ',');
    EXPECT_EQ(result1, split_result1);

    vector<string> result2 = {"a,b,v ,d , ,s"};
    EXPECT_EQ(split(str, '-'), result2);

    vector<string> result3 = {"a,b,", " ,d , ,s"};
    EXPECT_EQ(split(str, 'v'), result3);

    vector<string> result4 = {"a,b,v", ",d", ",", ",s"};
    EXPECT_EQ(split(str, ' '), result4);
    EXPECT_EQ(split(str), result4);
}

TEST(Common_Methods /*Testcase Name*/,Interval_Function /*Test name*/) {
    // * "a-d"   -> {a,b,c,d} AND
    // * "[a,d]" -> {a,b,c,d}
    typedef vector<string> INTER;
    INTER ints1a = create_interval("-3-6");
    INTER ints1b = {"-3", "-2", "-1", "0", "1", "2", "3", "4", "5", "6"};
    EXPECT_EQ(ints1a, ints1b);
    EXPECT_EQ(create_interval("[-3,6]"), ints1b);

    INTER ints2a =  create_interval("-4--1");
    INTER ints2b = {"-4", "-3", "-2", "-1"};
    EXPECT_EQ(ints2a, ints2b);
    EXPECT_EQ(create_interval("[-4,-1]"), ints2b);

    INTER ints3a =  create_interval("4--1");
    INTER ints3b = {};
    EXPECT_EQ(ints3a, ints3b);
    EXPECT_EQ(create_interval("[4,-1]"), ints3b);

    INTER ints4a =  create_interval("-4--5");
    INTER ints4b = {};
    EXPECT_EQ(ints4a, ints4b);
    EXPECT_EQ(create_interval("[-4,-5]"), ints4b);

    INTER ints5a = create_interval("4-2");
    INTER ints5b = {};
    EXPECT_EQ(ints5a, ints5b);
    EXPECT_EQ(create_interval("[4,2]"), ints5b);

    // * "a"     -> {a}
    INTER int6 = {"213"}; INTER int7 = {"2"};
    EXPECT_EQ(int6, create_interval("213"));
    EXPECT_EQ(int7, create_interval("2"));

    // * "a,b,d" -> {a,b,d}
    INTER int9 = {"2", "-5", "7", "0"};
    EXPECT_EQ(int9, create_interval("2,-5,7,0"));

    INTER ints10 = {"a", "c", "-T", "-sg 3"};
    EXPECT_EQ(ints10, create_interval("a,c,-T,-sg 3"));
}

TEST(Common_Methods /*Testcase Name*/,Permutation_Functions /*Test name*/) {
    std::vector<uint> v1 = {2,3,4,1,0};
    std::vector<uint> v2 = {0,1,2,3,4};
    std::vector<uint> v3 = {0,2,3,6,4,2};
    std::vector<uint> v4 = {0,1,2,3,5};
    EXPECT_TRUE(is_permutation(v1));
    EXPECT_TRUE(is_permutation(v2));
    EXPECT_FALSE(is_permutation(v3));
    EXPECT_FALSE(is_permutation(v4));

    for(int i=0; i<10;++i){
        auto vec = random_permutation(20, i);
            EXPECT_TRUE(is_permutation(vec));
    }
}    


TEST(Common_Methods /*Testcase Name*/,All_Combinations_Iterator /*Test name*/) {
    //test with a string
    {   
        std::string s = "12345";
        std::size_t comb_size = 3;
        std::set<string> all_combinations;
        do{
            all_combinations.insert( s.substr(0,comb_size) );      
        }
        while( next_combination(begin(s),begin(s) + comb_size,end(s)) );
        EXPECT_EQ(10/*5 choose 3*/, all_combinations.size());
    }

    {   
        std::string s = "234";
        std::size_t comb_size = 3;
        std::set<string> all_combinations;
        do{
            all_combinations.insert( s.substr(0,comb_size) );      
        }
        while( next_combination(begin(s),begin(s) + comb_size,end(s)) );
        EXPECT_EQ(1/*3 choose 3*/, all_combinations.size());
    }

    //test with a vector
    {
        std::vector<double> v = {1,2,3,4};
        std::size_t comb_size = 3;
        std::set<std::vector<double>> all_combinations;
        do{
            std::vector<double> selection(begin(v), begin(v) +comb_size);
            all_combinations.insert( selection );      
        }
        while( next_combination( begin(v),begin(v) + comb_size, end(v)) );
        EXPECT_EQ(4/*4 choose 3*/, all_combinations.size());
    }

}    