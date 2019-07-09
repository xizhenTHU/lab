from splinter.browser import Browser
import time
import datetime
import re


url = "https://scheduler.normbyte.com/account/sign_in"
x = Browser(driver_name="chrome")
x.visit(url)
# 填写登陆账户、密码
x.find_by_text(u"邮箱").click()
# x.fill("user[email]", "1021855443@qq.com")
x.fill("user[email]", "wangzongwei@pku.edu.cn")
x.find_by_text(u"密码").click()
# x.fill("user[password]", "qinyb369789")
x.fill("user[password]", "wangzongwei")
x.find_by_value('登录').click()
x.find_link_by_partial_href("/appointments/overview")[2].click()
x.find_link_by_partial_href("appointments/n").click()

# x.find_link_by_partial_href("start_date=2019-04-21").click()
if (datetime.date.today() + datetime.timedelta(1)).weekday() == 6:#如果预约抢的日期是周日
    ThisWeekSunday = str(
        datetime.date.today() +
        datetime.timedelta(
            6 -
            datetime.date.today().weekday()))
    x.find_link_by_partial_href("start_date=" + ThisWeekSunday).click()

day_grab = str(datetime.date.today() + datetime.timedelta(1))
print("准备抢预约的日期是" + day_grab)
aspiration = "45"  # 对应真实 3,4
day_today = str(datetime.date.today())
words = x.html

info_appoint = []
for m in re.finditer("已预约", words):
    day_appoint = words[m.start() - 17:m.start() - 7]  # [0:10]
    del_time = int(time.mktime(time.strptime(day_appoint, '%Y-%m-%d'))) - int(
        time.mktime(time.strptime(day_grab, '%Y-%m-%d')))
    order = int(words[m.start() - 4])  # :m.start() - 2][-2])
    if order == 's':  # 午夜前半
        order = 0
    info_appoint.append([del_time, order])

num_table = []
for jj in aspiration:
    num_temp = 0
    for ii in range(len(info_appoint)):
        num_temp += info_appoint[ii][1] <= int(jj)
    num_table.append(num_temp + int(jj))

num_word = words.find(
    "m-d" +
    day_today +
    "-es61")  # m-d2019-04-24-es41
point_num = 1300  # 2620#1408
# words = words[num_word + 1570 - 200:num_word + 1573 + 200]
# words = words[num_word + 1148 - 100:num_word + 1151 + 100]
words = words[num_word +
              point_num -
              200:num_word +
              point_num +
              200]  # while循环中也要改
# words = words[3772 - 350:3772 + 350]
while '今日 12:15 开放' in words:  # 今日 12:15 开放   #可预约(新)
    # time.sleep(0.15)
    x.reload()
    words = x.html
    num_word = words.find(
        "m-d" +
        day_today +
        "-es61")  # m-d2019-04-19-es21
    words = words[num_word + point_num - 200:num_word + point_num + 200]


num = 1
for ii in aspiration:
    str_day = "#m-d" + day_grab + "-es" + ii + "1"
    str_btn = x.find_link_by_partial_href(str_day)[0].text
    if '可预约' in str_btn:
        x.find_link_by_partial_href(str_day)[0].click()
        num_yuyue = num_table[num - 1]
        # if ii == '3':
        #     num_yuyue = 7 - 1
        # elif ii == '2':
        #     num_yuyue = 5 - 1
        # else:
        #     num_yuyue = 8 - 1
        try:
            time.sleep(0.17)
            x.find_by_name('commit')[num_yuyue].click()
            # x.find_by_value('提交')[1].click()
        except Exception:
            time.sleep(0.17)
            x.find_by_name('commit')[num_yuyue].click()
            # x.find_by_value('提交')[1].click()
        print("预约" + ii + "成功")
    else:
        print("预约" + ii + "失败" + str(num))
    num += 1


# x.find_element_by_xpath("//div/a[contains(text(), 新闻)]")
# with open("data.txt", "w", encoding='utf-8') as f:
#     f.write(x.html)  # 将字符串写入文件中
