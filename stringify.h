#pragma once
#include <string>
#include <functional>

template <typename T>
std::string add_numbers(const T& value){
    return "_"+std::to_string(value);
}

template <>
std::string add_numbers<std::string>(const std::string& value){
    return value;
}

template <typename U, typename... T>
std::string add_numbers(const U& head, const T&... tail)
{
    return add_numbers(head)+ add_numbers(tail...);
}

//some helper
template <typename T>
std::function<std::string (const T&)> make_stringify(const std::string s){
	return std::bind(add_numbers<std::string,T>, s,std::placeholders::_1);
}

template <typename T1, typename T2>
std::function<std::string (const T1&, const T2&)> make_stringify(const std::string s){
	return bind(add_numbers<std::string,T1,T2>, s,std::placeholders::_1,std::placeholders::_2);
}

template <typename T1, typename T2, typename T3>
std::function<std::string (const T1&, const T2&, const T3&)> make_stringify(const std::string s){
	return bind(add_numbers<std::string,T1,T2,T3>, s,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
}

//longer:
//std::function<std::string (int, int, int.....,int )> x = bind(add_numbers<std::string,int,...,int>, "x",std::placeholders::_1,std::placeholders::_2,...std::placeholders::_17);

/*
//how to use this:

auto t = make_stringify<float>("t");
auto x = make_stringify<int,int>("x");
auto y = make_stringify<int,int>("y");
auto f = bind(add_numbers<std::string,int,bool,char,char,char>, 
		 "f",
		 std::placeholders::_1,
		 std::placeholders::_2,
		 std::placeholders::_3,
		 std::placeholders::_4,
		 std::placeholders::_5);
		
int main(){
	using namespace std;
	cout << t(3) 					<< endl;
	cout << x(1,2)			 		<< endl;
	cout << y(2,3) 					<< endl;
	cout << f(3,true,'a','b','c') 	<< endl;


	return 0;
}
*/

