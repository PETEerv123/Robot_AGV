# Robot AGV Project Structure

Dự án Robot AGV được tổ chức thành ba nhóm chính: **Code**, **Hardware** và **Tools** nhằm tách biệt phần mềm, phần cứng và các công cụ hỗ trợ phát triển.

* **Code**

  * Chứa các chương trình kiểm thử (Unit Test) phục vụ việc đánh giá từng chức năng độc lập của hệ thống.
  * `Unit_Test_Speed`: Kiểm thử thuật toán điều khiển và đo tốc độ động cơ.
  * `Unit_Test_Quadrature_Encoder`: Kiểm thử chức năng đọc và xử lý tín hiệu Encoder Quadrature.

* **Hardware**

  * Chứa các tài liệu liên quan đến thiết kế và tích hợp phần cứng.
  * `Mechanical_Assembly`: Bản vẽ cơ khí, mô hình lắp ráp và hướng dẫn lắp đặt.
  * `System_Diagram.png`: Sơ đồ tổng thể mô tả kiến trúc hệ thống AGV, bao gồm các khối phần cứng và kết nối giữa chúng.

* **Tools**

  * Chứa mã nguồn hệ thống và các công cụ hỗ trợ phát triển.
  * `System_Code`: Mã nguồn chính của Robot AGV, bao gồm các module điều khiển, giao tiếp, xử lý cảm biến và các chức năng vận hành của hệ thống.
