"""
    pyproj算法复杂度太高，程序在这里时间开销太大
    tip：初始化的内容放在模块的前部，在模块一加载的时候就进行初始化
    更新：6.26 替换ll2xy函数，使用cpp混合编程
    更新：10.10更换pyproj包的使用方式，速度提升极大。202110101748更新完毕
"""

from pyproj import Transformer

# UMT坐标有分区 注意北京是50 烟台是51


transformer_utm2ll = Transformer.from_crs("epsg:32650", "epsg:4326")
transformer_ll2utm = Transformer.from_crs("epsg:4326", "epsg:32650")



def lonlat2utm(lon, lat):

    utm_x, utm_y = transformer_ll2utm.transform(lat, lon)  # 经纬度是反着的 是纬度再经度 平均调用一次0.03ms
    return utm_x, utm_y

def utm2lonlat(utm_x, utm_y):
    # lon, lat = transform(p, WGS84, x, y)
    lat, lon = transformer_utm2ll.transform(utm_x, utm_y)
    return lon, lat

def test_func():
    lon = 116.35604 #经度东经，经度范围0-180
    lat = 40.00643  #纬度北纬，纬度范围0-90

    x = 445036.7208032364
    y = 4428669.456629878

    print(lonlat2utm(lon,lat))
    print(utm2lonlat(x,y))
    print(" lon = 116.35604 #经度东经，经度范围0-180,\n lat = 40.00643   #纬度北纬，纬度范围0-90, \n 对应xy为445036.7208032364 4428669.456629878,\n cpp test data:445036.720803445037 UTMN=4428669.456630445037")



if __name__ == "__main__":
    test_func()
    pass

