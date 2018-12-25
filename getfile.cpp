#include"getfile.h"

//获取字符串出中的数字
int getNumberInString(string &istring, bool &hasnumbr)
{
	int number = 0;
	string filterstring;

	for (int i = istring.size(); i > 0 && istring[i - 1] != '_'&& istring[i - 1] != '/'&&istring[i - 1] != '\\'; i--)
	{
		if (istring[i - 1] >= '0'&&istring[i - 1] <= '9')
			filterstring.insert(filterstring.begin(), istring[i - 1]);
	}
	//QMessageBox::information(nullptr, "info", QString::fromStdString(filterstring));
	/*std::stringstream temp;
	temp << filterstring;
	temp >> number;*/
	number = atoi(filterstring.c_str());
	if (number == 0)
		hasnumbr = false;
	return number;
}

//根据文件名字中指定数字大小从小到大排序
template <class T>
int reRangeFileName(vector<T>& files, vector<int> &SerialNumber)
{
	if (files.size() != SerialNumber.size())
	{
		//cout << "The number of Files and Serial Number is wrong" << endl;
		return -1;
	}
	for (int i = 0; i < files.size(); i++)
	{
		for (int j = i; j < files.size(); j++)
		{
			if (SerialNumber[i] >= SerialNumber[j])
			{
				//swap(SerialNumber[i], SerialNumber[j]);

				int tmp;
				tmp = SerialNumber[j];
				SerialNumber[j] = SerialNumber[i];
				SerialNumber[i] = tmp;
				//swap(files[i], files[j]);
				T tmpname;
				tmpname = files[j];
				files[j] = files[i];
				files[i] = tmpname;
			}
		}

	}
	return 0;
}

//获取指定文件夹下特定格式的文件,其中files是输出文件名称vector
int GetAllFiles_CertainFormat(string path, vector<string>& files, string format)
{
	intptr_t hFile = 0;//_findnext返回类型为intprt_t,而非long类型，从intptr_t转换到long类型丢失数据
	struct  _finddata_t fileinfo;
	vector<int> SerialNumber;//用于存储序号
	//string serialnumber;//用于排序
	bool hasnumber = true;//用于判断文件名中是否有数字
	int nonum = 0;//用于在没有数字时排序
	string p;
	if ((hFile = _findfirst(p.assign(path).append("/*" + format).c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib & _A_SUBDIR))//判断是否为文件夹
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, ".") != 0)
				{
					GetAllFiles_CertainFormat((p.assign(path).append("/")).append(fileinfo.name), files, format);

				}
			}
			else
			{
				string serialnumber;//用于排序
				serialnumber.append(fileinfo.name);
				//QMessageBox::information(nullptr, "info", QString::fromStdString(serialnumber));
				//QMessageBox::information(nullptr, "info", fileinfo.name);
				int num = getNumberInString(serialnumber, hasnumber);
				//cout << "number:" << num << endl;
				if (hasnumber == true)
				{
					SerialNumber.push_back(num);
					//QMessageBox::information(nullptr, "info", "has number");
				}
				else
				{
					SerialNumber.push_back(nonum);
					//QMessageBox::information(nullptr, "info", "no number");
				}
				p.assign(path).append("/").append(fileinfo.name);
				files.push_back(p);
			}
			nonum++;
			//serialnumber.clear();

		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
	reRangeFileName(files, SerialNumber);
	//cout << "-------------------------------------------------" << endl;
	return 0;
}

//获取文件后缀名，即文件名称
string getpostfixname(string file_name)
{
	string postname;
	for (auto i = file_name.end() - 1; *i != '/'; i--)
	{
		postname.insert(postname.begin(), *i);
	}
	//第二种写法
	//for (int i = file_name.size(); file_name[i] != '/'; i--)
	//{
	//	postname.insert(postname.begin(), filename[i]);
	//}
	return postname;
}