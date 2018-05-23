#!/usr/bin/env python
# -*- coding: utf-8 -*

import rospy
from std_msgs.msg import String,Bool
import os
import time
import Image
import PIL
from reportlab.pdfgen import canvas
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont

class PDFGenerator:
    def __init__(self):
#        self.pdf_req_sub = rospy.Subscriber('/pdf_req',String,self.RequestCB)
        self.pdf_create_sub = rospy.Subscriber('/pdf/create_req',Bool,self.Create)
        self.pdf_append_first_sub = rospy.Subscriber('/pdf/append/first_req',Bool,self.AppendFirst)
        self.pdf_append_second_sub = rospy.Subscriber('/pdf/append/second_req',Bool,self.AppendSecond)

        self.pdf_result_pub = rospy.Publisher('/pdf_result',Bool,queue_size=1)
        
#        self.pdf_c = canvas.Canvas('HappyMimi_StoringGroceries_1.pdf')
        self.pdf_c = canvas.Canvas('HappyMimi_StoringGroceries')
        self.font_name = "HeiseiKakuGo-W5" # or "HeiseiMin-W3"
        self.create_count = 0

    def Create(self, *_):
        self.create_count += 1
        count = String()
        count.data = str(self.create_count)
        canvas_name = String()
        canvas_name.data = 'HappyMimi_StoringGroceries_' + count.data + '.pdf'
        self.pdf_c = canvas.Canvas(canvas_name.data)
        os.system('convert /home/nvidia/catkin_ws/src/e_object_recognizer/images/object_detection_result.png -quality 100 ~/catkin_ws/src/e_object_recognizer/images/result.jpg')                                                                 
        image = Image.open('/home/nvidia/catkin_ws/src/e_object_recognizer/images/result.jpg')                                                
        self.pdf_c.setPageSize((image.size[0]+10,image.size[1]*2))      
#        self.pdf_c.drawInlineImage(image,0,image.size[1])               

        #page 1                                                          
        pdfmetrics.registerFont(UnicodeCIDFont(self.font_name))
        self.pdf_c.setFont(self.font_name,45)
        self.pdf_c.drawString(100,460, u"Happy Mimi")
        self.pdf_c.drawString(100,400, u"Recognition Report")
        self.pdf_c.showPage()
        self.pdf_c.save()
        #page 2                                                          
        self.pdf_c.setFont(self.font_name,25)
        self.pdf_c.drawString(30,900, u"Team Name : Happy Mini")
        self.pdf_c.drawString(30,850, u"The Try Number : " + count.data)
        self.pdf_c.drawString(30,800, u"The date and time : "+time.ctime())
        self.pdf_c.drawString(30,750, u"Picture of the cupboard in its initial state:")
        self.pdf_c.drawInlineImage(image, 5,image.size[1]-240)          
        self.pdf_c.showPage()
        self.pdf_c.save()
        result = Bool()
        result.data = True
        self.pdf_result_pub.publish(result)
        print 'Created!'

    def AppendFirst(self,*_):
        os.system('convert /home/nvidia/catkin_ws/src/e_object_recognizer/images/object_detection_result.png -quality 100 ~/catkin_ws/src/e_object_recognizer/images/result.jpg')

        image = Image.open('/home/nvidia/catkin_ws/src/e_object_recognizer/images/result.jpg')
        pdfmetrics.registerFont(UnicodeCIDFont(self.font_name))
        self.pdf_c.setFont(self.font_name,25)
        self.pdf_c.drawString(30,750, u"Picture of the cupboard in its initial state:")
        self.pdf_c.drawInlineImage(image, 5, image.size[1]-240)
        self.pdf_c.showPage()
        result = Bool()
        result.data = True
        self.pdf_result_pub.publish(result)
        self.pdf_c.save()
        print 'Appended!'
        
        
    def AppendSecond(self,*_):
        os.system('convert /home/nvidia/catkin_ws/src/e_object_recognizer/images/object_detection_result.png -quality 100 ~/catkin_ws/src/e_object_recognizer/images/result2.jpg')
        image = Image.open('/home/nvidia/catkin_ws/src/e_object_recognizer/images/result2.jpg')
        pdfmetrics.registerFont(UnicodeCIDFont(self.font_name))
        self.pdf_c.setFont(self.font_name,25)
        self.pdf_c.drawString(30,750, u"Picture of manipulated object on cupboard:")
        self.pdf_c.drawInlineImage(image, 5, image.size[1]-240)
        self.pdf_c.showPage()
        self.pdf_c.save()
        result = Bool()
        result.data = True
#        os.system('mv /home/demulab/sample.pdf /USB_ROOT')
        self.pdf_result_pub.publish(result)
        print 'SecondAppended!'


if __name__ == '__main__':
    rospy.init_node('PDF_Generator',anonymous=True)
    pg = PDFGenerator()
    print 'Waiting'
    rospy.spin()
