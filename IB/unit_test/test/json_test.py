import json

target_pos = [
    1,
    2,
    3,
]

file_path = "/Users/kawasakiayumu/ARLISS_IBIS/IB/unit_test/test/test.json"

try:
    with open(file_path, mode="r") as f:
        target_pos_dict = json.load(f)
    
    target_pos_dict["location_name"] = target_pos
    
    with open(file_path, mode="w") as f:
        json.dump(target_pos_dict, f, indent=4) 
        
    print("target_pos_dictに新しいデータを追加しました。")
except FileNotFoundError:
    print("ファイルが見つかりません。")
except json.JSONDecodeError:
    print("ファイルが正しいJSON形式ではありません。")
except Exception as e:
    print("エラーが発生しました:", e)
