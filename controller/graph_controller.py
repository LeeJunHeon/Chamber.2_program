import numpy as np
import pyqtgraph as pg
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QVBoxLayout

class GraphController:
    def __init__(self, rga_widget, oes_widget):
        font = QFont()
        font.setPointSize(12)

        # === RGA 그래프 설정 ===
        self.rga_plot = pg.PlotWidget()
        self.rga_plot.setBackground('w')
        self.rga_plot.setLabel('left', 'Partial Pressure', color='k')
        self.rga_plot.setLabel('bottom', 'Atomic Mass', color='k')
        self.rga_plot.showGrid(x=False, y=True)
        self.rga_plot.setLogMode(y=False)
        self.rga_plot.setMouseEnabled(x=False, y=False)
        self.rga_plot.setYRange(-11, -5)
        self.rga_plot.setXRange(0, 65)

        self.rga_plot.getAxis('left').setStyle(tickFont=font)
        self.rga_plot.getAxis('bottom').setStyle(tickFont=font)

        y_axis_rga = self.rga_plot.getAxis('left')
        log_ticks = [[(-i, f'1E{-i}') for i in range(5, 12)]]
        y_axis_rga.setTicks(log_ticks)
        y_axis_rga.enableAutoSIPrefix(False)

        x_axis_rga = self.rga_plot.getAxis('bottom')
        x_ticks = [[(i, str(i)) for i in range(0, 66, 5)]]
        x_axis_rga.setTicks(x_ticks)

        for axis in ['left', 'bottom']:
            self.rga_plot.getAxis(axis).setTextPen('k')
            self.rga_plot.getAxis(axis).setPen(pg.mkPen('k'))

        # ▼▼▼ [핵심 수정] 스템 그래프를 그리기 위한 가장 안정적인 ErrorBarItem 사용 ▼▼▼
        # 1. 점선 (Stem) 부분
        pen_style = pg.mkPen(color='r', width=1, style=pg.QtCore.Qt.PenStyle.DotLine)
        self.rga_stem_item = pg.ErrorBarItem(x=np.array([]), y=np.array([]),
                                             height=np.array([]), beam=0,
                                             pen=pen_style)
        self.rga_plot.addItem(self.rga_stem_item)

        # 2. 점 (Marker) 부분
        self.rga_marker_item = pg.PlotDataItem(pen=None, symbol='s', symbolSize=6, symbolBrush='r')
        self.rga_plot.addItem(self.rga_marker_item)
        # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

        rga_layout = QVBoxLayout(rga_widget)
        rga_layout.addWidget(self.rga_plot)
        rga_widget.setLayout(rga_layout)

        # === OES 그래프 설정 (기존과 동일) ===
        self.oes_plot = pg.PlotWidget()
        self.oes_plot.setBackground('w')
        self.oes_plot.setLabel('left', 'Intensity', color='k')
        self.oes_plot.setLabel('bottom', 'Wavelength (nm)', color='k')
        self.oes_plot.showGrid(x=False, y=True)
        self.oes_plot.setMouseEnabled(x=False, y=False)
        self.oes_plot.setXRange(100, 1200)
        self.oes_plot.setYRange(0, 16000)
        x_axis_oes = self.oes_plot.getAxis('bottom')
        x_ticks = [[(i, str(i)) for i in range(100, 1300, 100)]]
        x_axis_oes.setTicks(x_ticks)
        y_axis_oes = self.oes_plot.getAxis('left')
        y_ticks = [[(i, str(i)) for i in range(0, 18000, 2000)]]
        y_axis_oes.setTicks(y_ticks)
        y_axis_oes.enableAutoSIPrefix(False)
        for axis in ['left', 'bottom']:
            ax = self.oes_plot.getAxis(axis)
            ax.setStyle(tickFont=font)
            ax.setTextPen('k')
            ax.setPen(pg.mkPen('k'))
        oes_layout = QVBoxLayout(oes_widget)
        oes_layout.addWidget(self.oes_plot)
        oes_widget.setLayout(oes_layout)
        self.oes_curve = self.oes_plot.plot(pen=pg.mkPen('r', width=1))

    def update_rga_plot(self, x_data, y_data):
        # 1. 데이터 유효성 검사 및 변환
        x_data = np.array(x_data, dtype=float)
        y_data = np.array(y_data, dtype=float)

        # 의도하신 대로 음수 값은 제외
        valid_mask = y_data >= 0
        x_data_valid = x_data[valid_mask]
        y_data_valid = y_data[valid_mask]

        if len(x_data_valid) == 0:
            self.rga_stem_item.setData(x=np.array([]), y=np.array([]), height=np.array([]))
            self.rga_marker_item.setData(x=np.array([]), y=np.array([]))
            return

        # 2. 로그 변환
        y_data_clipped = np.clip(y_data_valid, 1e-12, None)
        log_y_data = np.log10(y_data_clipped)

        # ▼▼▼ [핵심 수정] ErrorBarItem에 맞게 데이터 설정 ▼▼▼
        # 막대의 상단(top)과 하단(bottom)을 정의
        top_of_bar = log_y_data
        # ▼▼▼ [핵심 수정] 막대의 시작점을 그래프의 Y축 최소값으로 설정 ▼▼▼
        bottom_of_bar = np.full_like(log_y_data, self.rga_plot.getViewBox().viewRange()[1][0])

        # ErrorBarItem은 막대의 '중심점(y)'과 '총 높이(height)'를 사용
        bar_centers_y = (top_of_bar + bottom_of_bar) / 2
        bar_heights = top_of_bar - bottom_of_bar

        # 3. 그래프 데이터 업데이트
        # 스템(수직선) 업데이트
        self.rga_stem_item.setData(x=x_data_valid, y=bar_centers_y, height=bar_heights)
        # 마커(점) 업데이트
        self.rga_marker_item.setData(x=x_data_valid, y=top_of_bar)
        # ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲

    def update_oes_plot(self, x_data, y_data):
        self.oes_curve.setData(x_data, y_data)
