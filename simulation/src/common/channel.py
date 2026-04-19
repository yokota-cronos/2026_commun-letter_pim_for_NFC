from dataclasses import dataclass, field


@dataclass
class Packet:
    sender_id: int
    start_tu: int
    end_tu: int
    kind: str  # "inventory_cmd", "eof", "tag_response", "data_request", "data_response", "preamble", "postamble", "start_signal"


class Channel:
    def __init__(self):
        """
        チャネルの初期化
        
        アクティブなパケットと衝突したパケットのIDを管理するデータ構造を初期化します。
        """
        self._active_packets: list[Packet] = []
        self._collided_ids: set[int] = set()

    def submit(self, packet: Packet):
        """
        チャネルにパケットを送信
        
        Args:
            packet: 送信するパケット
        """
        self._active_packets.append(packet)

    def get_active_packets(self, current_tu: int) -> list[Packet]:
        """
        指定された時刻でアクティブなパケット一覧を取得
        
        Args:
            current_tu: 現在のTime Unit（TU）
            
        Returns:
            指定時刻で送受信中のパケット一覧
        """
        return [p for p in self._active_packets if p.start_tu <= current_tu < p.end_tu]

    def check_collision(self, current_tu: int) -> tuple[list[Packet], bool]:
        """
        指定された時刻でパケットの衝突をチェック
        
        複数のタグが同時に送信している場合、衝突と判定し、
        該当するパケットIDを記録します。
        
        Args:
            current_tu: 現在のTime Unit（TU）
            
        Returns:
            (アクティブなパケット一覧, 衝突フラグ)
        """
        active = self.get_active_packets(current_tu)
        tag_packets = [p for p in active if p.sender_id >= 0]  # reader has id -1
        collision = len(tag_packets) > 1
        if collision:
            for p in tag_packets:
                self._collided_ids.add(id(p))
        return active, collision

    def is_collided(self, packet: Packet) -> bool:
        """
        指定されたパケットが衝突したかどうかを判定
        
        Args:
            packet: 判定対象のパケット
            
        Returns:
            パケットが衝突した場合True、そうでない場合False
        """
        return id(packet) in self._collided_ids

    def cleanup(self, current_tu: int):
        """
        指定された時刻より前に終了したパケット情報をクリーンアップ
        
        メモリ使用量を削減するため、既に終了したパケットと
        その関連情報を削除します。
        
        Args:
            current_tu: 現在のTime Unit（TU）
        """
        self._active_packets = [p for p in self._active_packets if p.end_tu > current_tu]
        self._collided_ids = {pid for pid in self._collided_ids
                              if any(id(p) == pid for p in self._active_packets)}

    def clear(self):
        """
        チャネルの全データをクリア
        
        新しいラウンド開始時に呼び出され、前ラウンドのパケット情報と
        衝突情報を全削除します。
        """
        self._active_packets.clear()
        self._collided_ids.clear()
