# include <iostream>
# include <fstream>
# include <cmath>
# include <sstream>
# include <string>

#include <vector>
# include <filesystem>

using namespace std;


/**
* @brief フォルダ以下のファイル一覧を取得する関数
* @param[in]    folderPath  フォルダパス
* @param[out]   file_names  ファイル名一覧
*/
void get_csv_FileNames(std::string folder_Path, std::vector<std::string> &file_names){
    using namespace std::filesystem;
    directory_iterator iter(folder_Path), end;
    std::error_code err;

    for (; iter != end && !err; iter.increment(err)) {
        const directory_entry entry = *iter;

        file_names.push_back( entry.path().string() );
        printf("%s\n", file_names.back().c_str());
    }

    /* エラー処理 */
    if (err) {
        std::cout << err.value() << std::endl;
        std::cout << err.message() << std::endl;
    }
}



    /**
     *  @brief 目的のcsvを探す関数
     */
void find(string path_input, string path_output, double threshold){


    std::ifstream ifs_csv_file(path_input);  // csvファイルを開く





}


int main(){
    cout << "実行中..." << endl;




    return 0;
}