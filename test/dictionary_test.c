#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

struct node {
    struct node* left;
    struct node* right;
    char data;
};

struct node*
make_new_node() {
    struct node* new_node = (struct node*)malloc(sizeof(struct node));
    new_node->data        = '\0';
    new_node->left        = NULL;
    new_node->right       = NULL;
    return new_node;
}

void
print_tree(struct node* root) {
    if (root == NULL) {
        return;
    }
    print_tree(root->left);
    printf("%c ", root->data);
    print_tree(root->right);
}

void
destroy_tree(struct node* root) {
    if (root == NULL) {
        return;
    }
    destroy_tree(root->left);
    destroy_tree(root->right);
    free(root);
}

int
node_move_left(struct node* nd) {
    if (nd != NULL & nd->left != NULL) {
        nd = nd->left;
        return 0;
    }
    return -1;
}

int
node_move_right(struct node* nd) {
    if (nd != NULL && nd->right != NULL) {
        nd = nd->right;
        return 0;
    }
    return -1;
}

int
node_traverse(struct node* nd, int val) {
    if (val == 0) {
        return node_move_left(nd);
    } else if (val == 1) {
        return node_move_right(nd);
    } else {
        return -1;
    }
}

struct node*
construct_tree(uint64_t* dict, int dict_len) {
    // 	for each item in dict:
    // 		extract code, length, char from item
    //		descend tree to proper node
    //		if its the last bit in code:
    //			set node->char = char

    struct node* root = make_new_node();

    for (int i = 0; i < dict_len; i++) {
        uint64_t encoding = dict[i];
        // continue if the entry is empty
        if (encoding == 0) {
            continue;
        }

        char character  = (char)(encoding & 0xFF);
        uint64_t length = (encoding >> 8) & 0xFF;
        uint64_t code   = encoding >> 17;

        // descend from the root
        struct node* cur = root;
        for (int i = (int)length - 1; i >= 0; i--) {
            uint64_t bit = (code >> i) & 1;

            if (bit == 0) {
                if (cur->left == NULL) {
                    cur->left = make_new_node();
                }
                cur = cur->left;
            } else {
                if (cur->right == NULL) {
                    cur->right = make_new_node();
                }
                cur = cur->right;
            }
            if (i == 0) {
                cur->data = (char)character;
            }
        }
    }

    return root;
}

#define INPUT                                                                                                          \
    {                                                                                                                  \
        918304, 225970977, 2754092, 226364205, 13895470, 3618901808, 14476382513, 57906434866, 28952236595,            \
            28952367668, 115812734005, 57904993078, 57905124151, 28953023032, 28952629817, 1809321530, 226233147,      \
            28051519, 904793439, 1311841, 2623074, 5768803, 3016036, 131941, 5113446, 6817383, 525416, 787561,         \
            225577834, 5769067, 2360684, 7079533, 1049710, 918639, 3016304, 225446769, 1138, 132211, 1574004, 7210613, \
            5900150, 5899895, 112855672, 4982393, 452201594, 0,                                                        \
    }
#define DICT_SIZE 128

void populate_read_dict(uint64_t* final_dict, uint64_t* raw_dict);

int
main() {
    uint64_t out_dict[DICT_SIZE] = { 0 };
    uint64_t in_dict[]           = INPUT;

    populate_read_dict(out_dict, in_dict);

    int fd = open("default_dict", O_WRONLY | O_CREAT | O_TRUNC);
    if (fd == -1) {
        printf("error opening file\n");
        exit(-1);
    }
    write(fd, "[", 1);
    // Just for testing that it works
    for (int i = 0; i < DICT_SIZE; i++) {
        char buf[50];
        int len = sprintf(buf, "%lu, ", out_dict[i]);
        write(fd, buf, len);

        if (out_dict[i] == 0) {
            continue;
        }
        char character  = (char)(out_dict[i] & 0xFF);
        uint64_t length = (out_dict[i] >> 8) & 0xFF;
        uint64_t code   = out_dict[i] >> 17;

        if (length > 64) {
            printf("error: length to large: %lu\n", length);
            return 1;
        }

        printf("char: %c\tlength: %lu\t", character, length);
        for (int j = (int)length - 1; j >= 0; j--) {
            printf("%lu", (code >> j) & 1);
        }
        printf("\n");
    }
    write(fd, "]", 1);
    close(fd);

    // test building huffman tree
    printf("\n\n\nHuffman tree test\n");
    struct node* tree_root = construct_tree(out_dict, DICT_SIZE);
    print_tree(tree_root);
    destroy_tree(tree_root);
    printf("\n");

    return 0;
}

void
populate_read_dict(uint64_t* final_dict, uint64_t* raw_dict) {
    for (int i = 0; raw_dict[i] != 0; i++) {
        uint64_t character = raw_dict[i] & 0xFF;
        if (character >= DICT_SIZE) {
            printf("error\n");
        } else {
            final_dict[character] = raw_dict[i];
        }
    }
}