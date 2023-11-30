#coding=utf-8
# -*- coding: UTF-8 -*-
#！/usr/bin/eny python
#coding:utf-8
#!/usr/bin/env python
# -*- coding:utf-8 -*-
import tornado.ioloop
import tornado.web
class MainHandler(tornado.web.RequestHandler):
    def get(self):
       self.render('g.html')
    def post(self,*args,**kwargs):
        x='X'
        y='Y'
        g='G01'
        k=0
        a1=0
        e=''
        a=int(self.get_argument('x'))
        b=int(self.get_argument('y'))
        c=int(self.get_argument('r'))
        d=int(self.get_argument('l'))
        j1=int(self.get_argument('f'))
        q=(b/2)+6
        k=(-(a/2))
        while k<a:
            a1=a1=1
            i=a1%2
            if i==1:
                k=k+(c/2)
            q=q*-1
            l=g+x+str(k)+y+str(q)
            e=e+l+'</br>'
        c1=-(a/2)
        c2=(b/2)
        l1='G00'+str(c1)+y+str(c2)+'</br>'+"Z5"+'</br>'
        l2=g+'Z'+str(d)+'F'+str(j1)+'</br>'
        l3=str(q)
        l4=g+'Y'+l3+'F'+str(j1)+'</br>'
        tou=l1+l2+l4
        w1='G00'+'Z20'+'</br>'
        w2='G00'+'X0Y0'
        zhong=tou+e+w1+w2
        self.write(zhong)
 
settings={
    }
application = tornado.web.Application([
    (r"/index", MainHandler),
],)
if __name__ == "__main__":
    application.listen(8000)
    tornado.ioloop.IOLoop.instance().start()