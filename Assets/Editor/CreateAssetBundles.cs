
using UnityEditor;
using System.IO;
public class CreateAssetBundles
{
    //设为一个菜单选项
    [MenuItem("Assets/Build AssetBundles")]
    static void BuildAllAssetBundles()
    {
        //创建一个文件路径
        string dir = "AssetBundles";
        if (Directory.Exists(dir) == false)
        {
            Directory.CreateDirectory(dir);
        }
        //输出路径,BuildAssetBundleOptions,平台
        BuildPipeline.BuildAssetBundles("AssetBundles", BuildAssetBundleOptions.None, BuildTarget.StandaloneWindows64);

    }
}

