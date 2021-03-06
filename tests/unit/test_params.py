#!/usr/bin/env python
# -*- coding: utf-8 -*-
import unittest
from collections import namedtuple, OrderedDict

from ddt import ddt, data, unpack

import rosunit

from fiware_ros_gopigo.params import get_params, find_item, TYPE


class TestGetParams(unittest.TestCase):
    def test_empty_dict(self):
        params = OrderedDict()
        result = get_params(params)
        self.assertEqual(result, namedtuple(TYPE, ())())

        with self.assertRaises(AttributeError):
            result.foo

    def test_simple_dict(self):
        params = OrderedDict(
            [('a', 'A'), ]
        )
        result = get_params(params)
        self.assertEqual(result, namedtuple(TYPE, ('a'))(a='A'))
        self.assertEqual(result.a, 'A')

    def test_nested_dict(self):
        params = OrderedDict(
            [
                ('a', OrderedDict(
                    [
                        ('aa', 'aA'),
                        ('ab', OrderedDict([('aba', 'abA'), ])),
                    ]
                )),
                ('b', 'B'),
            ]
        )
        result = get_params(params)
        ab = namedtuple(TYPE, ('aba'))(aba='abA')
        a = namedtuple(TYPE, ('aa', 'ab'))(aa='aA', ab=ab)
        self.assertEqual(result, namedtuple(TYPE, ('a', 'b'))(a=a, b='B'))
        self.assertEqual(result.a.aa, 'aA')
        self.assertEqual(result.a.ab.aba, 'abA')
        self.assertEqual(result.b, 'B')

    def test_simple_list(self):
        result = get_params(['a', 'b'])
        self.assertEqual(result, ['a', 'b'])

    def test_nested_list(self):
        params = [
            OrderedDict([('a', 'A0'), ]),
            OrderedDict([('a', 'A1'), ]),
            'B',
        ]
        result = get_params(params)
        a_type = namedtuple(TYPE, ('a'))
        self.assertEqual(result, [a_type(a='A0'), a_type(a='A1'), 'B'])
        self.assertEqual(result[0].a, 'A0')
        self.assertEqual(result[1].a, 'A1')
        self.assertEqual(result[2], 'B')

    def test_complex(self):
        params = OrderedDict(
            [
                ('a', 'A'),
                ('b', [
                    OrderedDict([('ba', 'bA0'), ('bb', 'bB0'), ]),
                    OrderedDict([('ba', 'bA1'), ('bb', 'bB1'), ]), ]),
            ]
        )
        result = get_params(params)
        b_type = namedtuple(TYPE, ('ba', 'bb'))
        self.assertEqual(result, namedtuple(TYPE, ('a', 'b'))
                         (a='A', b=[b_type(ba='bA0', bb='bB0'), b_type(ba='bA1', bb='bB1')]))
        self.assertEqual(result.a, 'A')
        self.assertEqual(result.b[0].ba, 'bA0')
        self.assertEqual(result.b[0].bb, 'bB0')
        self.assertEqual(result.b[1].ba, 'bA1')
        self.assertEqual(result.b[1].bb, 'bB1')


@ddt
class TestFindItem(unittest.TestCase):
    def setUp(self):
        super(TestFindItem, self).setUp()
        params = OrderedDict(
            [
                ('a', [OrderedDict([('key', 'KEY1'), ('value', 'VALUE1'), ]),
                       OrderedDict([('key', 'KEY2'), ('value', 'VALUE2'), ]), ]),
            ]
        )
        self.d = get_params(params)
        self.a_type = namedtuple(TYPE, ('key', 'value'))
        self.assertEqual(self.d, namedtuple(TYPE, ('a'))(a=[self.a_type(key='KEY1', value='VALUE1'),
                                                            self.a_type(key='KEY2', value='VALUE2')]))

    @unpack
    @data(*[{'obj': obj, 'key_name': key_name, 'key_value': key_value}
            for obj in ('a', 'invalid', 0, ['foo', 'bar'], {'x': 'y'}, None)
            for key_name in ('key', 0, ['foo', 'bar'], {'x': 'y'}, None)
            for key_value in ('KEY1', 0, ['foo', 'bar'], {'x': 'y'}, None)])
    def test_find_none1(self, obj, key_name, key_value):
        if obj == 'a':
            a = self.d.a
        else:
            a = obj

        result = find_item(a, key_name, key_value)
        if obj == 'a' and key_name == 'key' and key_value == 'KEY1':
            self.assertEqual(result, self.a_type(key='KEY1', value='VALUE1'))
            self.assertEqual(result.key, 'KEY1')
            self.assertEqual(result.value, 'VALUE1')
        else:
            self.assertIsNone(result)


if __name__ == '__main__':
    rosunit.unitrun('fiware_ros_gopigo', 'test_params', TestGetParams)
    rosunit.unitrun('fiware_ros_gopigo', 'test_params', TestFindItem)
