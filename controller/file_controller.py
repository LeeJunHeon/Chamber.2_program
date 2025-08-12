# file_controller.py
from PyQt6.QtWidgets import QFileDialog, QMessageBox

def select_files_or_folder(parent=None):
    """
    파일 또는 폴더를 선택하는 대화 상자.
    :param parent: 부모 QWidget
    :return: 선택된 파일 리스트 또는 폴더 경로 (문자열)
    """
    msg = QMessageBox(parent)
    msg.setWindowTitle("선택 유형")
    msg.setText("파일을 선택하시겠습니까, 폴더를 선택하시겠습니까?")
    file_button = msg.addButton("파일 선택", QMessageBox.ButtonRole.AcceptRole)
    folder_button = msg.addButton("폴더 선택", QMessageBox.ButtonRole.ActionRole)
    msg.addButton("취소", QMessageBox.ButtonRole.RejectRole)
    msg.exec()

    selected_button = msg.clickedButton()

    if selected_button == file_button:
        # 여러 파일 선택
        file_paths, _ = QFileDialog.getOpenFileNames(
            parent,
            "여러 파일 선택",
            "",
            "All Files (*);;Text Files (*.txt);;CSV Files (*.csv)"
        )
        return file_paths if file_paths else None

    elif selected_button == folder_button:
        # 폴더 선택
        folder_path = QFileDialog.getExistingDirectory(
            parent,
            "폴더 선택",
            ""
        )
        return folder_path if folder_path else None

    return None
