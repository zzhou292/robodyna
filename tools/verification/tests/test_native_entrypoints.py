import unittest

from tools.verification.native_entrypoints import entrypoints


class NativeEntrypointTests(unittest.TestCase):
    def test_real_definition_keeps_line_and_ignores_examples(self):
        source = '// int main() {}\nconst char* x = "int main() {}";\nint main(int argc, char** argv) { return 0; }\n'
        self.assertEqual(entrypoints(source), [dict(name='main', line=3, lexical_brace_depth=0)])

    def test_raw_strings_comments_and_prototype_are_not_programs(self):
        source = 'auto text = R"tag(int main() { fake(); })tag";\n/* int main() {} */\nint main(int, char**);\nvoid main_helper() {}\n'
        self.assertEqual(entrypoints(source), [])

    def test_nonstandard_filename_is_irrelevant_to_definitions(self):
        source = 'int WINAPI WinMain(void* instance) { return 0; }\nauto main() noexcept -> int { return 0; }\n'
        self.assertEqual([row['name'] for row in entrypoints(source)], ['WinMain', 'main'])

    def test_conditionals_remain_visible_and_nested_scope_is_flagged(self):
        source = '#if FIRST\nint main() {}\n#else\nint main() {}\n#endif\nstruct Helper { int main() {} };\n'
        self.assertEqual([row['lexical_brace_depth'] for row in entrypoints(source)], [0, 0, 1])


if __name__ == '__main__':
    unittest.main()
