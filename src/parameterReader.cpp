#include "parameterReader.h"

cParameterReader::cParameterReader(string fileName)
{
    ifstream parameterFile(fileName.c_str());
    if (!parameterFile)
    {
        cerr << "Do't find this file, pleas check your file's path and name" << endl;
        return;
    }
    while (!parameterFile.eof())
    {
        string str;
        getline(parameterFile, str);
        if (str[0] == '#') //#开头是注释
            continue;
        int pos = str.find("="); //找到=的位置
        if (pos == -1)
            continue;
        string key = str.substr(0, pos); //关键字
        string value = str.substr(pos + 1, str.length());
        data[key] = value;
        if (!parameterFile.good())
            break;
    }
    parameterFile.close();

    getData<double>("camera.fx", camera.fx);
    getData<double>("camera.fy", camera.fy);
    getData<double>("camera.cx", camera.cx);
    getData<double>("camera.cy", camera.cy);
    getData<double>("camera.d0", camera.d0);
    getData<double>("camera.d1", camera.d1);
    getData<double>("camera.d2", camera.d2);
    getData<double>("camera.d3", camera.d3);
    getData<double>("camera.d4", camera.d4);
    getData<double>("camera.scale", camera.factor);

    getData<string>("cfgPath", cfgPath);
    getData<string>("weightsPath", weightsPath);
    getData<string>("namePath", namePath);

    getData<bool>("topicOrFile", topicOrFile);
    getData<bool>("publishPointCloud", publishPointCloud);
    getData<bool>("publishImageBox", publishImageBox);
    getData<bool>("saveImageBox", saveImageBox);

    getData<string>("rgbTopic", rgbTopic);
    getData<string>("depthTopic", depthTopic);
    getData<string>("markerTopic", markerTopic);
    getData<string>("pointCloudTopic", pointCloudTopic);
    getData<string>("imageBoxTopic", imageBoxTopic);
    if (saveImageBox)
        getData<string>("savePath", savePath);
    initTum();
    getClassName();
}

template <class T>
void cParameterReader::getData(const string &key, T &value)
{
    map<string, string>::iterator iter = data.find(key);
    if (iter == data.end())
    {
        cerr << "Parameter data " << key << " not found" << endl;

        return;
    }
    try
    {

        value = boost::lexical_cast<T>(iter->second);
        // cout << value <<endl;
        return;
    }
    catch (boost::bad_lexical_cast &e)
    {
        cout << key << " Exception catched." << endl;
    }

    return;
}

void cParameterReader::initTum()
{
    getData<string>("imagesPath", imagesPath);

    getData<string>("datasetPath", datasetPath);
    string associateFile = imagesPath;
    ifstream fin(associateFile.c_str());
    if (!fin)
    {
        cerr << "找不到assciate.txt！在tum数据集中是必须的文件。" << endl;
        cerr << "请用python assicate.py rgb.txt depth.txt > associate.txt生成一个associate文件！" << endl;
        return;
    }

    while (!fin.eof())
    {
        string rgbTime, rgbFile, depthTime, depthFile;
        fin >> rgbTime >> rgbFile >> depthTime >> depthFile;
        if (!fin.good())
        {
            break;
        }
        rgbFiles.push_back(datasetPath + rgbFile);
        depthFiles.push_back(datasetPath + depthFile);
    }
    fin.close();
    // cout << "一共找到" << rgbFiles.size() << "个数据记录。" << endl;
    return;
}

void cParameterReader::getClassName()
{
    std::ifstream ifs(this->namePath.c_str());
    if (!ifs)
    {
        cerr << "找不着name.txt！请按照顺序添加权重对应的名字文件。" << endl;
        return;
    }
    std::string line;
    while (getline(ifs, line))
        this->className.push_back(line);
    this->num_class = this->className.size();
    ifs.close();
    // cout << "一共找到" << className.size() << "名字" << endl;
    return;
}