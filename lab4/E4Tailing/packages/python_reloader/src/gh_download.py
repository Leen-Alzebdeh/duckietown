# from https://gist.github.com/pdashford/2e4bcd4fc2343e2fd03efe4da17f577d

import base64
import shutil
from github import GithubException
import os


def get_sha_for_tag(repository, tag):
    """
    Returns a commit PyGithub object for the specified repository and tag.
    """
    branches = repository.get_branches()
    matched_branches = [match for match in branches if match.name == tag]
    if matched_branches:
        return matched_branches[0].commit.sha

    tags = repository.get_tags()
    matched_tags = [match for match in tags if match.name == tag]
    if not matched_tags:
        raise ValueError('No Tag or Branch exists with that name')
    return matched_tags[0].commit.sha


def download_directory(repository, sha, root_folder_path, root_content_path):
    """
    Download all contents at server_path with commit tag sha in
    the repository.
    """

    download_sub_folder(repository, sha, root_folder_path, root_content_path, '')
    print(f'download finished at {root_folder_path}')


def download_sub_folder(repository, sha, root_folder_path, root_content_path, sub_content_path):
    """
    Download all contents at server_path with commit tag sha in
    the repository.
    """
    if sub_content_path == '':
        content_path = root_content_path
    else:
        content_path = root_content_path + '/' + sub_content_path
    contents = repository.get_contents(content_path, ref=sha)
    target_path = root_folder_path + '/' + sub_content_path
    if type(contents) is list:
        if not os.path.exists(target_path):
            os.makedirs(target_path)
        for content in contents:
            download_sub_folder(repository, sha, root_folder_path, root_content_path, content.path[len(root_content_path) + 1:])
    else:
        content = contents
        try:
            file_data = base64.b64decode(content.content)
            file_out = open(target_path, 'wb+')
            file_out.write(file_data)
            file_out.close()
        except (GithubException, IOError) as exc:
            print(f'Error processing {content.path}')
            raise exc


def download_from_github(org_or_user, repo, branch, target_folder, root_content_path):
    """
    download a content of directory or file from github into the target folder
    """
    repository = org_or_user.get_repo(repo)
    sha = get_sha_for_tag(repository, branch)
    download_directory(repository, sha, target_folder, root_content_path)


